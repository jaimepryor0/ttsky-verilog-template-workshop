"""
vocoder_fixed_point.py — Fixed-point vocoder using hw_int and filter_df2_hw.

Pipeline:
  1. Load audio: hw_int (ADC_BITS wide, Q1.DATA_FRAC)
  2. Apply IIR bandpass filters to audio signal
  3. Envelope-detect each audio band: abs() + first-order IIR LP smoother
  4. Generate 500 Hz sawtooth: hw_int, apply same 3 bandpass filters
  5. Multiply each sawtooth band by its corresponding audio envelope
  6. Accumulate 3 products = output
"""

import numpy as np
import scipy.signal as sig
import soundfile as sf
import matplotlib.pyplot as plt

from hw_int import hw_int
from filter_df2 import filter_df2_hw

# ── Configuration ─────────────────────────────────────────────────────────────
FS        = 48_000
ADC_BITS  = 8
DATA_FRAC = 7       # Q1.7: audio in (-1, 1), 12-bit ADC truncated to top 8 bits
COEF_BITS = 8
COEF_FRAC = 6       # Q2.6: keeps SOS biquot a[] within (-2, 2); coarse but stable
SAW_FREQ  = 500     # Hz

# Voice bands: log-spaced to match vocal tract resonance structure
#   Band 1: fundamentals and lower formants    80 – 500 Hz
#   Band 2: core vowel formant region        500 – 2000 Hz
#   Band 3: fricatives, sibilance, upper F3 2000 – 6000 Hz
VOICE_BANDS = [
    (200,   1000),  # fundamentals + lower formants
    (500,  2000),   # core vowel formant region
]

FILTER_ORDER = 1    # Butterworth order; bandpass transform doubles it → 1 SOS section

# Envelope LP smoother: ~100 Hz cutoff. At Q2.6 the original 20 Hz cutoff
# would quantise b0 = (1 - alpha) to literal 0 -- filter dies. 100 Hz gives
# b0=1, a1=-63 at Q2.6 (effective cutoff ~120 Hz). Faster envelope tracking
# than before, but still well below voice band-limit.
ENV_FC    = 100.0
ENV_ALPHA = float(np.exp(-2.0 * np.pi * ENV_FC / FS))


# ── Helpers ───────────────────────────────────────────────────────────────────

def _wrap_coeffs(c: np.ndarray) -> hw_int:
    return hw_int.from_float(c, bits=COEF_BITS, frac_bits=COEF_FRAC)


def _design_bandpass(flo: float, fhi: float) -> list[tuple[hw_int, hw_int]]:
    """
    Butterworth bandpass → list of quantised biquad (b, a) pairs (SOS cascade).
    SOS keeps each section's a[] within (-2, 2), avoiding direct-form sensitivity.
    """
    sos = sig.butter(FILTER_ORDER, [flo, fhi], btype='bandpass', output='sos', fs=FS)
    return [(_wrap_coeffs(row[:3]), _wrap_coeffs(row[3:])) for row in sos]


def _filter_sos(sections: list[tuple[hw_int, hw_int]], x: hw_int) -> hw_int:
    """Cascade x through a list of biquad sections in order."""
    y = x
    for b_hw, a_hw in sections:
        y = filter_df2_hw(b_hw, a_hw, y)
    return y


def _envelope_detect(band: hw_int) -> hw_int:
    """
    Full-wave rectify then smooth with a one-pole IIR LP:
      y[n] = ENV_ALPHA * y[n-1] + (1 - ENV_ALPHA) * |x[n]|
    """
    rectified = abs(band)
    # First-order IIR: b=[1-alpha, 0], a=[1, -alpha]
    b_env = _wrap_coeffs(np.array([1.0 - ENV_ALPHA, 0.0]))
    a_env = _wrap_coeffs(np.array([1.0,            -ENV_ALPHA]))
    z = filter_df2_hw(b_env, a_env, rectified)
    return z


def _generate_sawtooth(n_samples: int, freq: float = SAW_FREQ) -> hw_int:
    """Rising sawtooth in (-1, 1), quantised to DATA_FRAC hw_int.

    `freq` defaults to SAW_FREQ. The chip's `pitch_byte` (ui_in) maps to
    a frequency via  f = FS * pitch_byte / 256  (see pitch.v).
    """
    t = np.arange(n_samples) / FS
    saw = sig.sawtooth(2.0 * np.pi * freq * t, width=1)
    return hw_int.from_float(saw, bits=ADC_BITS, frac_bits=DATA_FRAC)


# ── Debug helper ──────────────────────────────────────────────────────────────

def save_hw(x: hw_int, path: str, fs: int = FS) -> None:
    """Write an hw_int signal to a WAV file and print a one-line summary."""
    f = x.to_float().astype(np.float32)
    peak = float(np.abs(f).max())
    rms  = float(np.sqrt(np.mean(f.astype(np.float64)**2)))
    int_bits = x.bits - x.frac_bits - 1
    print(f"  {path:45s}  Q{int_bits}.{x.frac_bits} ({x.bits}b)  "
          f"peak={peak:.4f}  rms={rms:.4f}")
    sf.write(path, f, fs)


# ── Main pipeline ─────────────────────────────────────────────────────────────

def process(audio_hw: hw_int,
            saw_hw: hw_int | None = None,
            dump: bool = False,
            dump_dir: str = 'debug_out') -> hw_int:
    """
    Run the fixed-point vocoder chain on a Q1.15 audio signal and return the
    16-bit Q1.15 output. This is the bit-exact reference for vocoder.v.

    audio_hw : Q1.15 hw_int input (mic signal).
    saw_hw   : Q1.15 sawtooth to use as the carrier. Auto-generated from
               len(audio_hw) if None.
    dump     : if True, writes every intermediate stage as a WAV into dump_dir.
    """
    import os

    def _dump(x: hw_int, name: str) -> None:
        if dump:
            os.makedirs(dump_dir, exist_ok=True)
            save_hw(x, os.path.join(dump_dir, name + '.wav'))

    if saw_hw is None:
        saw_hw = _generate_sawtooth(len(audio_hw))
    _dump(audio_hw, '00_audio_in')
    _dump(saw_hw,   '03_sawtooth_raw')

    products: list[hw_int] = []
    for i, (flo, fhi) in enumerate(VOICE_BANDS, start=1):
        sections = _design_bandpass(flo, fhi)
        mic_band = _filter_sos(sections, audio_hw)
        env      = _envelope_detect(mic_band)
        saw_band = _filter_sos(sections, saw_hw)
        product  = (env * saw_band).truncate(ADC_BITS, DATA_FRAC)
        _dump(mic_band, f'01_mic_band{i}_{flo}-{fhi}Hz')
        _dump(env,      f'02_env_band{i}_{flo}-{fhi}Hz')
        _dump(saw_band, f'04_saw_band{i}_{flo}-{fhi}Hz')
        _dump(product,  f'05_product_band{i}_{flo}-{fhi}Hz')
        products.append(product)

    # Sum the per-band env*saw products and truncate back to the data
    # format -- matches vocoder.v's modular accumulation into `out`.
    acc = products[0]
    for p in products[1:]:
        acc = acc + p
    acc = acc.truncate(ADC_BITS, DATA_FRAC)
    _dump(acc, '06_output')
    return acc


def load_audio(audio_path: str) -> hw_int:
    """Load a WAV file, downmix to mono, resample to FS, wrap as Q1.15 hw_int."""
    audio, file_fs = sf.read(audio_path, dtype='float32', always_2d=False)
    if audio.ndim > 1:
        audio = audio[:, 0]
    if file_fs != FS:
        n_out = int(round(len(audio) * FS / file_fs))
        audio = sig.resample(audio, n_out)
    return hw_int.from_float(audio, bits=ADC_BITS, frac_bits=DATA_FRAC)


def run(audio_path: str, dump_dir: str | None = None) -> hw_int:
    """Load audio from a path and run it through process()."""
    audio_hw = load_audio(audio_path)
    return process(audio_hw, dump=dump_dir is not None, dump_dir=dump_dir or 'debug_out')


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import os, argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('audio', nargs='?', default='hello.wav')
    parser.add_argument('--debug', metavar='DIR',
                        help='dump every intermediate stage as WAV into DIR')
    args = parser.parse_args()

    if args.debug:
        os.makedirs(args.debug, exist_ok=True)
        print(f"Debug intermediates → {args.debug}/")

    print(f"Processing {args.audio} ...")
    result = run(args.audio, dump_dir=args.debug)

    out_path = args.audio.rsplit('.', 1)[0] + '_vocoder_hw.wav'
    save_hw(result, out_path)
    print(f"Saved → {out_path}")

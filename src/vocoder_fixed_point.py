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
ADC_BITS  = 16
DATA_FRAC = 15      # Q1.15: audio in (-1, 1), maps to full ADC range
COEF_BITS = 16
COEF_FRAC = 14      # SOS biquad a[] always within (-2, 2); b[] down to ~7e-4 at Q2.14
SAW_FREQ  = 500     # Hz

# Voice bands: log-spaced to match vocal tract resonance structure
#   Band 1: fundamentals and lower formants    80 – 500 Hz
#   Band 2: core vowel formant region        500 – 2000 Hz
#   Band 3: fricatives, sibilance, upper F3 2000 – 6000 Hz
VOICE_BANDS = [
    (200,   1000), #TODO
    (500,  2000),
    (2000, 6000),
]

FILTER_ORDER = 1    # Butterworth order; bandpass transform doubles it → 1 SOS section

# Envelope LP smoother: ~20 Hz cutoff, one-pole IIR
ENV_FC    = 20.0
ENV_ALPHA = float(np.exp(-2.0 * np.pi * ENV_FC / FS))  # ≈ 0.9974


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


def _generate_sawtooth(n_samples: int) -> hw_int:
    """Rising 500 Hz sawtooth in (-1, 1), quantised to DATA_FRAC hw_int."""
    t = np.arange(n_samples) / FS
    saw = sig.sawtooth(2.0 * np.pi * SAW_FREQ * t, width=1)
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

def run(audio_path: str, dump_dir: str | None = None) -> hw_int:
    """
    dump_dir: if set, writes every intermediate stage as a WAV into that directory.
    """
    import os

    def _dump(x: hw_int, name: str) -> None:
        if dump_dir is not None:
            save_hw(x, os.path.join(dump_dir, name + '.wav'))

    # Load and (if needed) resample to FS
    audio, file_fs = sf.read(audio_path, dtype='float32', always_2d=False)
    if audio.ndim > 1:
        audio = audio[:, 0]
    if file_fs != FS:
        n_out = int(round(len(audio) * FS / file_fs))
        audio = sig.resample(audio, n_out)

    n_samples = len(audio)

    # Wrap audio as hw_int
    audio_hw = hw_int.from_float(audio, bits=ADC_BITS, frac_bits=DATA_FRAC)
    _dump(audio_hw, '00_audio_in')

    # Design filters, filter audio, envelope-detect each band
    audio_envs: list[hw_int] = []
    band_sections: list[list[tuple[hw_int, hw_int]]] = []

    for i, (flo, fhi) in enumerate(VOICE_BANDS):
        sections = _design_bandpass(flo, fhi)
        band_sections.append(sections)

        mic_band = _filter_sos(sections, audio_hw)
        _dump(mic_band, f'01_mic_band{i+1}_{flo}-{fhi}Hz')
        
        env = _envelope_detect(mic_band)
        _dump(env, f'02_env_band{i+1}_{flo}-{fhi}Hz')
        audio_envs.append(env)

    # Generate sawtooth, apply same bandpass filters
    saw_hw = _generate_sawtooth(n_samples)
    _dump(saw_hw, '03_sawtooth_raw')

    saw_bands: list[hw_int] = []
    for i, (sections, (flo, fhi)) in enumerate(zip(band_sections, VOICE_BANDS)):
        saw_band = _filter_sos(sections, saw_hw)
        _dump(saw_band, f'04_saw_band{i+1}_{flo}-{fhi}Hz')
        saw_bands.append(saw_band)

    # Multiply each sawtooth band by its audio envelope, truncate back to data format
    products: list[hw_int] = []
    for i, (env, saw_band, (flo, fhi)) in enumerate(zip(audio_envs, saw_bands, VOICE_BANDS)):
        product = (env * saw_band).truncate(ADC_BITS, DATA_FRAC)
        _dump(product, f'05_product_band{i+1}_{flo}-{fhi}Hz')
        products.append(product)

    # Accumulate — each __add__ adds one guard bit (ADC_BITS → +2 after two sums)
    acc = products[0] + products[1]
    acc = acc + products[2]
    _dump(acc, '06_output')
    return acc


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
    result = result.truncate(16, 15)

    out_path = args.audio.rsplit('.', 1)[0] + '_vocoder_hw.wav'
    save_hw(result, out_path)
    print(f"Saved → {out_path}")

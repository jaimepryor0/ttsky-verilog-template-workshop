import numpy as np
import scipy.signal as sig


def vocoder_hq(x, fs,
               n_bands=24,
               fmin=80,
               fmax=8000,
               carrier_notes=None,
               attack_ms=5,
               release_ms=80,
               filter_order=2):
    """
    High-quality channel vocoder for reference/comparison.

    Parameters
    ----------
    x              : input signal (1D or column vector from soundfile)
    fs             : sample rate in Hz
    n_bands        : number of frequency bands (24 gives clear speech)
    fmin/fmax      : frequency range of the filter bank
    carrier_notes  : list of frequencies (Hz) for the chord carrier.
                     Defaults to a C major chord [261, 330, 392].
                     Use a single value like [120] for a plain robot voice.
    attack_ms      : envelope follower attack time constant in ms
    release_ms     : envelope follower release time constant in ms
    filter_order   : Butterworth order per band (each bandpass has 2*order poles)

    Returns
    -------
    out : ndarray, normalised to peak 0.9
    """
    if carrier_notes is None:
        carrier_notes = [261.63, 329.63, 392.00]  # C major chord (C4, E4, G4)

    x = np.asarray(x, dtype=np.float64).flatten()
    n = len(x)

    # --- Filter bank: contiguous log-spaced bands ---
    edges = np.geomspace(fmin, fmax, n_bands + 1)
    sos_banks = [
        sig.butter(filter_order, [edges[i], edges[i+1]], btype='band', fs=fs, output='sos')
        for i in range(n_bands)
    ]

    # --- Asymmetric envelope follower ---
    a = np.exp(-1 / (attack_ms  * 1e-3 * fs))
    r = np.exp(-1 / (release_ms * 1e-3 * fs))

    def envelope(band_sig):
        env = np.zeros(n)
        for i in range(1, n):
            v = abs(band_sig[i])
            c = a if v > env[i-1] else r
            env[i] = c * env[i-1] + (1 - c) * v
        return env

    # --- Voiced/unvoiced detection via zero-crossing rate ---
    frame = int(0.02 * fs)
    crossings = (np.diff(np.sign(x), prepend=0) != 0).astype(float)
    zcr = np.convolve(crossings, np.ones(frame) / frame, mode='same')
    zcr /= (zcr.max() + 1e-9)
    voiced = np.clip(1.0 - (zcr - 0.15) / 0.3, 0.0, 1.0)

    # --- Carrier: chord of sawtooths for voiced, noise for unvoiced ---
    t_ax = np.arange(n) / fs
    chord = sum(sig.sawtooth(2 * np.pi * f * t_ax, width=0.5) for f in carrier_notes)
    chord /= len(carrier_notes)  # normalise by number of notes
    noise  = np.random.randn(n)
    noise *= np.sqrt(np.mean(chord**2)) / (np.sqrt(np.mean(noise**2)) + 1e-9)
    carrier = voiced * chord + (1 - voiced) * noise

    # --- Synthesis ---
    out = np.zeros(n)
    for sos in sos_banks:
        mic_band = sig.sosfilt(sos, x)
        env      = envelope(mic_band)
        car_band = sig.sosfilt(sos, carrier)
        out     += env * car_band

    peak = np.max(np.abs(out))
    if peak > 0:
        out *= 0.9 / peak

    return out

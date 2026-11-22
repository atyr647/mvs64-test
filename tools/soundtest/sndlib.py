"""Shared helpers for the sound reference tests (numpy only)."""
import re, numpy as np

YM_RATE = 8000000 / 144          # YM2610 FM/ADPCM sample rate (ymfm "MED" fidelity)
OUT_RATE = 22050                 # our output rate

def load(path):
    return np.fromfile(path, '<i2').astype(np.float64)

def resample(x, src, dst):
    """Band-limited (FFT) resample."""
    n_out = int(round(len(x) * dst / src))
    X = np.fft.rfft(x)
    keep = n_out // 2 + 1
    Y = np.zeros(keep, complex); m = min(keep, len(X)); Y[:m] = X[:m]
    return np.fft.irfft(Y, n_out) * (n_out / len(x))

def highpass(x, a=0.9943):
    """1-pole DC blocker (~20 Hz at 22050)."""
    y = np.empty_like(x); px = py = 0.0
    for i, v in enumerate(x):
        py = a * (py + v - px); px = v; y[i] = py
    return y

def best_corr(a, b, maxlag=40):
    best = (-2.0, 0)
    for lag in range(-maxlag, maxlag + 1):
        bb = np.roll(b, lag)
        c = np.dot(a, bb) / (np.linalg.norm(a) * np.linalg.norm(bb) + 1e-9)
        if c > best[0]: best = (c, lag)
    return best

def rms(x):
    return float(np.sqrt((x ** 2).mean())) if len(x) else 0.0

def spectrum(x, n=4096):
    acc = np.zeros(n // 2 + 1)
    for i in range(0, len(x) - n, n // 2):
        acc += np.abs(np.fft.rfft(x[i:i + n] * np.hanning(n))) ** 2
    return acc, np.fft.rfftfreq(n, 1 / OUT_RATE)

def band_shares(x, bands):
    s, f = spectrum(x)
    return [100 * s[(f >= a) & (f < b)].sum() / s.sum() for a, b in bands]

def log_spec_corr(x, y, lo=100, hi=8000):
    (sx, f), (sy, _) = spectrum(x), spectrum(y)
    m = (f > lo) & (f < hi)
    return float(np.corrcoef(np.log10(sx[m] + 1), np.log10(sy[m] + 1))[0, 1])

def peaks(x, count=5, minf=60, sep=25):
    s, f = spectrum(x); out = []
    for i in np.argsort(s)[::-1]:
        if f[i] < minf: continue
        if all(abs(f[i] - q) > sep for q in out): out.append(f[i])
        if len(out) == count: break
    return sorted(int(round(v)) for v in out)

def peak_freq(x, rate=OUT_RATE):
    """Strongest partial in Hz, ~0.05 Hz precision (Hann, 16x zero-pad, parabolic fit)."""
    n = len(x); N = 1 << int(np.ceil(np.log2(n * 16)))
    s = np.log(np.abs(np.fft.rfft(x * np.hanning(n), N)) + 1e-9)
    i = int(np.argmax(s[1:-1])) + 1
    d = 0.5 * (s[i - 1] - s[i + 1]) / (s[i - 1] - 2 * s[i] + s[i + 1])
    return (i + d) * rate / N

def wav_from_log(path):
    """Rebuild the [WAV] hex capture (SND_SELFTEST build) from an ares log."""
    out = []
    for line in open(path, errors='replace'):
        if line.startswith('[WAV] '):
            h = line[6:].strip()
            if re.fullmatch(r'[0-9a-f]+', h):
                out += [(int(h[i:i + 4], 16) ^ 0x8000) - 0x8000 for i in range(0, len(h), 4)]
    return np.array(out, float)

class Checks:
    def __init__(self): self.fail = 0
    def check(self, name, ok, detail):
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
        if not ok: self.fail += 1

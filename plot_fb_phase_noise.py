import numpy as np
import pandas as pd
from numpy.fft import rfft, rfftfreq
import matplotlib.pyplot as plt

df = pd.read_csv("fb_xbit_fall_times.csv")

# tie_s is the cumulative timing error. jitter_s is the per-cycle period error.
tie = df["tie_s"].to_numpy() if "tie_s" in df.columns else df["jitter_s"].to_numpy()
t_noisy = df["t_noisy_s"].to_numpy()

T_avg = np.mean(np.diff(t_noisy))

phi = 2 * np.pi * tie / T_avg
phi -= np.mean(phi)

fs = 1.0 / T_avg

w = np.hanning(len(phi))
Phi = rfft(phi * w)

N = len(phi)

Sxx = (np.abs(Phi)**2) / (fs * np.sum(w**2))
Sxx[1:-1] *= 2

f = rfftfreq(N, d=1/fs)

L_dbc = 10 * np.log10(Sxx/2 + 1e-30)

# Cycle-by-cycle frequency from consecutive fall times
period = np.diff(t_noisy)
freq_hz = 1.0 / period
t_freq = t_noisy[1:]

# Plot
plt.figure(figsize=(10, 6))
plt.semilogx(f[1:], L_dbc[1:])

plt.xlabel("Frequency Offset (Hz)")
plt.ylabel("Phase Noise (dBc/Hz)")
plt.title("Phase Noise")

plt.grid(True, which="both")

plt.figure(figsize=(10, 6))
plt.plot(t_freq * 1e6, freq_hz / 1e9)
plt.xlabel("Time (us)")
plt.ylabel("Frequency (GHz)")
plt.title("Frequency vs Time")
plt.grid(True)

plt.show()

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. Load data
df = pd.read_csv('fb_xbit_fall_times.csv')

# 2. Scale jitter from seconds to picoseconds (ps)
jitter_ps = df['jitter_s'] * 1e12

# 3. Plot PDF using standard Matplotlib
fig, ax = plt.subplots(figsize=(9, 5))

# Plot normalized density histogram
ax.hist(
    jitter_ps, 
    bins=100, 
    density=True, 
    color="royalblue", 
    edgecolor="black", 
    alpha=0.6
)

ax.set_title("Probability Density Function (PDF) of Column D: jitter_s", fontsize=13, fontweight='bold')
ax.set_xlabel("Jitter (ps)", fontsize=11)
ax.set_ylabel("Probability Density (1/ps)", fontsize=11)
ax.grid(True, linestyle="--", alpha=0.5)

# 4. Display statistical metrics
stats_text = (
    f"Samples: {len(jitter_ps):,}\n"
    f"Mean: {jitter_ps.mean():.4f} ps\n"
    f"Std Dev (RMS): {jitter_ps.std():.4f} ps\n"
    f"Median: {jitter_ps.median():.4f} ps\n"
    f"Min: {jitter_ps.min():.2f} ps\n"
    f"Max: {jitter_ps.max():.2f} ps"
)
ax.text(0.96, 0.95, stats_text, transform=ax.transAxes, fontsize=10,
        verticalalignment='top', horizontalalignment='right',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.85, edgecolor='gray'))

plt.tight_layout()
plt.show()
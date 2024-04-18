import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(figsize=(12, 6), subplot_kw=dict(aspect="equal"))

labels = (
    # "Noise injection",
    "ADC to signals",
    "FFT",
    "Impedance computation",
    "Smoothing",
    # "Phase centering",
    "Neural network",
)
sizes = [168, 375, 5, 410, 9]  # 12,


def func(pct, allvals):
    absolute = int(np.round(pct / 100.0 * np.sum(allvals)))
    return f"{pct:.1f}%\n({absolute:d} ms)"


wedges, texts, autotexts = ax.pie(
    sizes,
    autopct=lambda dat: func(dat, sizes),
    textprops=dict(color=(0, 0, 0)),
    startangle=90,
    counterclock=False,
    pctdistance=0.65,
)

ax.legend(
    wedges,
    labels,
    title="Identification Steps",
    loc="center left",
    bbox_to_anchor=(1, 0, 0.5, 1),
)

plt.setp(autotexts, size=8, weight="bold")

ax.set_title(f"Computation time of an estimation : {sum(sizes)}ms")

plt.show()

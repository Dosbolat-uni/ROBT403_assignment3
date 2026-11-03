#!/usr/bin/env python3
"""Plot theta_ref vs theta_motor (and u) from a task4_control CSV log.
Usage: python3 plot_log.py task4_log.csv      -> task4_log.png
       python3 plot_log.py step_20.csv        -> step_20.png
"""
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "task4_log.csv"
t, ref, th, u, mode = [], [], [], [], []
with open(path) as f:
    for r in csv.DictReader(f):
        t.append(float(r["time_s"])); ref.append(float(r["theta_ref_deg"]))
        th.append(float(r["theta_motor_deg"])); u.append(float(r["u_percent"])); mode.append(r["mode"])

fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6), height_ratios=[3, 1.3])
# shade zones (three-zone runs only)
colors = {"ZONE1_STOP": "#e8e8e8", "ZONE2_PID": "#dcebfa", "ZONE3_FOLLOW": "#fbe5d6"}
start = 0
for i in range(1, len(t) + 1):
    if i == len(t) or mode[i] != mode[start]:
        if mode[start] in colors:
            a1.axvspan(t[start], t[i - 1], color=colors[mode[start]], lw=0)
        start = i
a1.plot(t, ref, label=r"$\theta_{ref}$ (IMU / step)", lw=1.6)
a1.plot(t, th, label=r"$\theta_{motor}$ (encoder)", lw=1.6)
for y in (10, -10, 30, -30):
    if any(m.startswith("ZONE") for m in mode):
        a1.axhline(y, color="0.6", lw=0.6, ls="--")
from matplotlib.patches import Patch
handles, _ = a1.get_legend_handles_labels()
if any(m in colors for m in mode):
    handles += [Patch(color=c, label=z.replace("_", " ").title()) for z, c in colors.items()]
a1.set_ylabel("angle (deg)"); a1.legend(handles=handles, loc="upper right", fontsize=8); a1.grid(alpha=0.3)
a2.plot(t, u, color="C2", lw=1.2); a2.set_ylabel("u (% duty)"); a2.set_xlabel("time (s)"); a2.grid(alpha=0.3)
a1.set_title(path)
fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=150)
print("saved", out)

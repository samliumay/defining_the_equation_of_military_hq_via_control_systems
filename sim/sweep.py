"""Full system (section 7): sweep link delay d and packet loss p for four arms.

Arms: local autonomy on/off x safety filter on/off, all under spoofing.
"""

import csv
import statistics
from itertools import product
from pathlib import Path

import matplotlib.pyplot as plt

from model import Params, run

ROOT = Path(__file__).resolve().parent.parent
DELAYS = [0.0, 1.0, 2.0, 4.0]
LOSSES = [0.0, 0.3, 0.6, 0.9]
SEEDS = [0, 1, 2]
ARMS = [(True, True), (True, False), (False, True), (False, False)]
SPOOF_RATE = 0.05  # one injected intent every ~20 s


def arm_name(autonomy, filt):
    return f"autonomy {'on' if autonomy else 'off'}, filter {'on' if filt else 'off'}"


def main():
    rows = []
    for (autonomy, filt), d, loss in product(ARMS, DELAYS, LOSSES):
        res = [
            run(Params(delay=d, loss=loss, local_autonomy=autonomy, safety_filter=filt,
                       spoof_rate=SPOOF_RATE, wind_gust=0.5), seed=s)
            for s in SEEDS
        ]
        recv = sum(r["malicious_received"] for r in res)
        rows.append({
            "arm": arm_name(autonomy, filt),
            "delay": d,
            "loss": loss,
            "mission_success": statistics.mean(r["mission_success"] for r in res),
            "outside_s": statistics.mean(r["outside_s"] for r in res),
            "malicious_rejected_pct": 100 * sum(r["malicious_rejected"] for r in res) / recv if recv else float("nan"),
        })
        print(f"{rows[-1]['arm']:28s} d={d:3.1f} p={loss:.1f}  success={rows[-1]['mission_success']:.2f}  "
              f"outside S={rows[-1]['outside_s']:5.1f} s  rejected={rows[-1]['malicious_rejected_pct']:5.1f} %")

    (ROOT / "results").mkdir(exist_ok=True)
    with open(ROOT / "results" / "sweep.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    (ROOT / "figures").mkdir(exist_ok=True)
    fig, axes = plt.subplots(2, 4, figsize=(18, 8))
    for col, (autonomy, filt) in enumerate(ARMS):
        name = arm_name(autonomy, filt)
        for row, (key, cmap, vmax, title) in enumerate([
            ("mission_success", "viridis", 1.0, "mission success"),
            ("outside_s", "magma_r", 90.0, "time outside safe set (s)"),
        ]):
            grid = [[next(r[key] for r in rows if r["arm"] == name and r["delay"] == d and r["loss"] == l)
                     for l in LOSSES] for d in DELAYS]
            ax = axes[row, col]
            im = ax.imshow(grid, cmap=cmap, vmin=0, vmax=vmax, origin="lower")
            for i, d in enumerate(DELAYS):
                for j, l in enumerate(LOSSES):
                    ax.text(j, i, f"{grid[i][j]:.2f}" if row == 0 else f"{grid[i][j]:.0f}",
                            ha="center", va="center", color="white" if row == 0 else "black", fontsize=8)
            ax.set_xticks(range(len(LOSSES)), LOSSES)
            ax.set_yticks(range(len(DELAYS)), DELAYS)
            ax.set(xlabel="packet loss p", ylabel="delay d (s)", title=f"{name}\n{title}")
            fig.colorbar(im, ax=ax, fraction=0.046)
    fig.tight_layout()
    fig.savefig(ROOT / "figures" / "sweep.png", dpi=110)
    print(f"Saved {ROOT / 'results' / 'sweep.csv'} and {ROOT / 'figures' / 'sweep.png'}")


if __name__ == "__main__":
    main()

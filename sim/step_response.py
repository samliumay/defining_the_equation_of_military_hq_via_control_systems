"""Inner loop only (sections 3-4): step responses, wind offset, PD vs PID."""

import math
from pathlib import Path

import matplotlib.pyplot as plt

from model import Autopilot, Params, plant_step

OUT = Path(__file__).resolve().parent.parent / "figures"


def simulate(p: Params, ref, t_end=8.0):
    s = [0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
    ap = Autopilot(p)
    ts, xs, zs, ths = [], [], [], []
    for k in range(round(t_end / p.dt)):
        T, tau = ap(s, ref)
        s = plant_step(s, T, tau, p.wind, p)
        ts.append(k * p.dt)
        xs.append(s[0])
        zs.append(s[1])
        ths.append(math.degrees(s[2]))
    return ts, xs, zs, ths


def main():
    OUT.mkdir(exist_ok=True)
    ref = (2.0, 7.0)  # 2 m sideways, 2 m up
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))

    for kpx, kdx in [(4, 4), (4, 1.6), (4, 8)]:
        zeta = kdx / (2 * math.sqrt(kpx))
        ts, xs, _, _ = simulate(Params(kpx=kpx, kdx=kdx), ref)
        ax[0].plot(ts, xs, label=f"Kp={kpx}, Kd={kdx} (ζ={zeta:.1f})")
    ax[0].axhline(ref[0], color="grey", ls=":")
    ax[0].set(title="Position x: damping ζ = Kd / (2√Kp)", xlabel="t (s)", ylabel="x (m)")
    ax[0].legend()

    ts, xs, zs, ths = simulate(Params(), ref)
    ax[1].plot(ts, zs, label="z (m)")
    ax[1].plot(ts, ths, label="θ (deg)")
    ax[1].axhline(ref[1], color="grey", ls=":")
    ax[1].set(title="Altitude and tilt (attitude is 5× faster)", xlabel="t (s)")
    ax[1].legend()

    wind = 1.5  # N
    p = Params(wind=wind)
    offset = wind / (p.m * p.kpx)
    for label, kix in [("PD", 0.0), ("PID (Ki=2)", 2.0)]:
        ts, xs, _, _ = simulate(Params(wind=wind, kix=kix), ref, t_end=15)
        ax[2].plot(ts, xs, label=label)
    ax[2].axhline(ref[0], color="grey", ls=":")
    ax[2].axhline(ref[0] + offset, color="red", ls="--", label=f"w/(m·Kpx) = {offset:.2f} m")
    ax[2].set(title=f"Constant wind w = {wind} N", xlabel="t (s)", ylabel="x (m)")
    ax[2].legend()

    fig.tight_layout()
    fig.savefig(OUT / "step_response.png", dpi=120)
    print(f"Predicted wind offset (PD): {offset:.3f} m")
    print(f"Simulated x at t=15 s:  PD {simulate(Params(wind=wind), ref, 15)[1][-1] - ref[0]:+.3f} m, "
          f"PID {simulate(Params(wind=wind, kix=2.0), ref, 15)[1][-1] - ref[0]:+.3f} m")
    print(f"Saved {OUT / 'step_response.png'}")


if __name__ == "__main__":
    main()

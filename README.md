# Defining the equation of a military HQ via control systems

Can a central HQ, connected to a fleet of autonomous nodes (drones, ground robots), be written down as one control system? This repo models it as a **cascaded (hierarchical) feedback system** with three nested loops, each slower and smarter than the one inside it:

| Loop | Runs on | Rate | Controller |
|------|---------|------|------------|
| Inner | Vehicle autopilot (PX4 / ArduPilot) | ~400 Hz | PID on the vehicle dynamics, no model |
| Middle | Node agent | ~10 Hz | Small onboard model + safety filter picks the next waypoint |
| Outer | HQ | ~1 Hz | Planner model reallocates tasks over a lossy, delayed radio link |

## Scope

- **Non-kinetic.** The reference mission is search-and-rescue / mapping in a disaster zone with simulated jamming. No targeting or weapon-engagement logic.
- **Human judgment stays in the loop.** HQ intents pass an operator approval screen (in line with DoDD 3000.09).
- **Security is the research angle:** spoofed HQ orders, a captured node, prompt injection through sensor data, model theft from a downed vehicle.

## Contents

| File | What it holds |
|------|---------------|
| [docs/system_equations.md](docs/system_equations.md) | The full model: plant, three loops, closed-loop form, linearisation |
| [docs/architecture.md](docs/architecture.md) | HQ / node design, control principles, MVP and the planned experiment |

## Next step

A small simulation of the three loops that sweeps link delay `d` and packet loss `p`, measuring mission completion with and without local autonomy and the safety filter.

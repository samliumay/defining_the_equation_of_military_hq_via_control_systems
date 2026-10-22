"""Three-loop model of the HQ / node / autopilot system.

Every block follows docs/system_equations.md; section numbers refer to that file.
The node and HQ "models" (pi_node, pi_HQ) are simple rule-based stand-ins, so the
control structure can be studied before any LLM is plugged in.
"""

import math
import random
from dataclasses import dataclass, field

G = 9.81
MAX_TILT = math.radians(30)


@dataclass
class Params:
    # Plant (section 1)
    m: float = 1.5  # kg
    J: float = 0.02  # kg m^2
    wind: float = 0.0  # constant wind force w, N
    wind_gust: float = 0.0  # std of random wind force, N
    # Inner loop gains (section 3); defaults give omega=2, zeta=1 and omega=10, zeta=1
    kpz: float = 4.0
    kdz: float = 4.0
    kpx: float = 4.0
    kdx: float = 4.0
    kix: float = 0.0  # integral term on x (PD -> PID)
    kpth: float = 100.0
    kdth: float = 20.0
    # Rates (timescale separation)
    dt: float = 1 / 400  # inner loop, 400 Hz
    node_period: float = 0.1  # middle loop, 10 Hz
    hq_period: float = 1.0  # outer loop, 1 Hz
    # Middle loop (section 5)
    sensor_noise: float = 0.1  # std of v, m
    carrot: float = 3.0  # max distance of p_ref ahead of the vehicle, m
    reach: float = 1.0  # a survey point counts as visited within this radius, m
    geofence: tuple = (0.0, 50.0, 2.0, 30.0)  # safe set S: x_min, x_max, z_min, z_max
    # Outer loop (section 6)
    delay: float = 0.0  # link delay d, s
    loss: float = 0.0  # packet loss probability p
    local_autonomy: bool = True  # node advances the plan itself
    t_out: float | None = None  # intent timeout; default depends on local_autonomy
    # Security
    safety_filter: bool = True  # signature check + Proj_S
    spoof_rate: float = 0.0  # injected malicious intents per second
    spoof_signed: float = 0.3  # fraction of injected intents signed with a stolen key
    # Mission
    t_max: float = 90.0
    plan: list = field(
        default_factory=lambda: [
            (5, 5), (15, 5), (25, 5), (35, 5), (45, 5),
            (45, 15), (35, 15), (25, 15), (15, 15), (5, 15),
        ]
    )

    def timeout(self) -> float:
        if self.t_out is not None:
            return self.t_out
        return 20.0 if self.local_autonomy else 3.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# Section 1: plant
def plant_step(s, T, tau, w, p: Params):
    x, z, th, vx, vz, om = s
    ax = (T * math.sin(th) + w) / p.m
    az = T * math.cos(th) / p.m - G
    al = tau / p.J
    # semi-implicit Euler
    vx += ax * p.dt
    vz += az * p.dt
    om += al * p.dt
    return [x + vx * p.dt, max(0.0, z + vz * p.dt), th + om * p.dt, vx, vz, om]


# Section 3: inner loop
class Autopilot:
    def __init__(self, p: Params):
        self.p = p
        self.ix = 0.0

    def __call__(self, s, ref):
        p = self.p
        x, z, th, vx, vz, om = s
        xr, zr = ref
        a_z = p.kpz * (zr - z) - p.kdz * vz
        T = clamp(p.m * (G + a_z) / math.cos(th), 0.0, 4 * p.m * G)
        ex = xr - x
        self.ix += ex * p.dt
        th_des = clamp((p.kpx * ex + p.kix * self.ix - p.kdx * vx) / G, -MAX_TILT, MAX_TILT)
        tau = p.J * (p.kpth * (th_des - th) - p.kdth * om)
        return T, tau


# Section 5: safety filter, Proj_S onto the geofence box
def proj_s(pt, p: Params):
    x0, x1, z0, z1 = p.geofence
    return clamp(pt[0], x0, x1), clamp(pt[1], z0, z1)


def inside_s(pt, p: Params, tol=0.5):
    x0, x1, z0, z1 = p.geofence
    return x0 - tol <= pt[0] <= x1 + tol and z0 - tol <= pt[1] <= z1 + tol


@dataclass
class Intent:
    plan: list  # waypoints; with local autonomy the whole plan, otherwise one waypoint
    signed: bool
    malicious: bool = False


class NodeAgent:
    """Section 5: pi_node (rule-based stand-in) + intent hold from section 6."""

    def __init__(self, p: Params):
        self.p = p
        self.intent: Intent | None = None
        self.intent_time = -math.inf
        self.visited: set = set()
        self.loiter_at = None
        self.stats = {"malicious_received": 0, "malicious_rejected": 0, "legit_rejected": 0}

    def receive(self, intent: Intent, t):
        if intent.malicious:
            self.stats["malicious_received"] += 1
        if self.p.safety_filter and not intent.signed:
            if intent.malicious:
                self.stats["malicious_rejected"] += 1
            else:
                self.stats["legit_rejected"] += 1
            return
        self.intent = intent
        self.intent_time = t

    def __call__(self, s, t):
        p = self.p
        o = (s[0] + random.gauss(0, p.sensor_noise), s[1] + random.gauss(0, p.sensor_noise))
        # mark survey points as visited (uses true position, the sensor is only for steering)
        for i, pt in enumerate(p.plan):
            if math.dist((s[0], s[1]), pt) < p.reach:
                self.visited.add(i)

        held_ok = self.intent is not None and t - self.intent_time < p.timeout()
        target = None
        if held_ok:
            self.loiter_at = None
            if self.intent.malicious or not p.local_autonomy:
                target = self.intent.plan[0]
            else:
                remaining = [pt for i, pt in enumerate(p.plan) if i not in self.visited]
                target = remaining[0] if remaining else None
        if target is None:  # I_safe: loiter where we are
            if self.loiter_at is None:
                self.loiter_at = o
            target = self.loiter_at

        # carrot: p_cand at most `carrot` metres ahead of the observed position
        dx, dz = target[0] - o[0], target[1] - o[1]
        dist = math.hypot(dx, dz)
        k = min(1.0, p.carrot / dist) if dist > 0 else 0.0
        p_cand = (o[0] + k * dx, o[1] + k * dz)
        return proj_s(p_cand, p) if p.safety_filter else p_cand


class HQ:
    """Section 6: pi_HQ (rule-based stand-in) working on delayed telemetry."""

    def __init__(self, p: Params):
        self.p = p
        self.visited_seen: set = set()

    def __call__(self, telemetry):
        if telemetry is not None:
            self.visited_seen = set(telemetry["visited"])
        remaining = [pt for i, pt in enumerate(self.p.plan) if i not in self.visited_seen]
        if not remaining:
            return None
        plan = remaining if self.p.local_autonomy else [remaining[0]]
        return Intent(plan=plan, signed=True)


class Link:
    """Radio link with delay d and loss probability p."""

    def __init__(self, p: Params):
        self.p = p
        self.queue = []

    def send(self, msg, t):
        if random.random() >= self.p.loss:
            self.queue.append((t + self.p.delay, msg))

    def deliver(self, t):
        out = [m for (ta, m) in self.queue if ta <= t]
        self.queue = [(ta, m) for (ta, m) in self.queue if ta > t]
        return out


def spoofed_intent(p: Params) -> Intent:
    # attacker tries to pull the vehicle out of the safe set (far away, near the ground)
    return Intent(plan=[(random.uniform(70, 100), 0.5)], signed=random.random() < p.spoof_signed, malicious=True)


# Section 7: the whole hybrid system
def run(p: Params, seed: int = 0, record: bool = False):
    random.seed(seed)
    s = [0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
    ap, node, hq = Autopilot(p), NodeAgent(p), HQ(p)
    down, up = Link(p), Link(p)  # HQ -> node, node -> HQ
    telemetry = None
    p_ref = (s[0], s[1])
    outside_time = 0.0
    trace = []
    node_every = round(p.node_period / p.dt)
    hq_every = round(p.hq_period / p.dt)
    steps = round(p.t_max / p.dt)

    for k in range(steps):
        t = k * p.dt
        if k % hq_every == 0:
            # node reports to HQ, HQ plans on the latest telemetry it has received
            up.send({"pos": (s[0], s[1]), "visited": sorted(node.visited)}, t)
            for msg in up.deliver(t):
                telemetry = msg
            intent = hq(telemetry)
            if intent is not None:
                down.send(intent, t)
        if p.spoof_rate > 0 and random.random() < p.spoof_rate * p.dt:
            node.receive(spoofed_intent(p), t)  # attacker transmits close to the node
        for msg in down.deliver(t):
            node.receive(msg, t)
        if k % node_every == 0:
            p_ref = node(s, t)
        w = p.wind + random.gauss(0, p.wind_gust)
        T, tau = ap(s, p_ref)
        s = plant_step(s, T, tau, w, p)
        if not inside_s((s[0], s[1]), p):
            outside_time += p.dt
        if record and k % 40 == 0:
            trace.append((t, *s, *p_ref))
        if len(node.visited) == len(p.plan):
            break

    st = node.stats
    result = {
        "mission_success": len(node.visited) / len(p.plan),
        "time": t,
        "outside_s": outside_time,
        "malicious_received": st["malicious_received"],
        "malicious_rejected": st["malicious_rejected"],
        "legit_rejected": st["legit_rejected"],
    }
    return (result, trace) if record else result

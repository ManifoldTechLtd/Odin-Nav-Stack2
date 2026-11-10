# NeuPAN Optimisation Notes

**English** | [中文](neuPAN_optimisation.zh-CN.md)

**NeuPAN** is one of ONS2's local planners: a penalty-based MPC that alternates between two
blocks every control tick:

- **DUNE** — maps the robot shape into a latent point space,
- **NRMP** — solves a nonlinear receding-horizon QP in that latent space.

Out of the box this costs far more than a 50 ms control period allows on embedded
hardware. The optimisations below are what make ONS2 viable on a Jetson-class machine.

---

## Measured performance

After the optimisations, same parameters, **single planning computation**:

| Platform | Single planning computation |
|---|---|
| Jetson Orin NX 8 GB | **30–40 ms** |
| Intel NUC (x86) | **15–30 ms** |

> This is the planner's own computation time and excludes perception, transport and the
> control chain. End-to-end tick time also depends on scene complexity, obstacle count and
> host load — measure it on your target machine with the planner's own timing logs.

---

## 1. Planner core

### 1.1 DUNE: neural encoder → analytic closed form

The original DUNE encoder ran a small MLP forward pass on every tick. Because the
network's activation is piecewise linear and the mapping is exercised over a fixed shape
set, the same mapping can be expressed **analytically**.

- Removed the per-tick inference entirely.
- Faster **and** more accurate: the analytic form reproduces the network's mapping to a
  much tighter tolerance than the network itself, so the latent geometry is cleaner.
- This is the single largest change in the DUNE path.
- **Trade-off:** the analytic form cannot self-tune the way the trained network could —
  real-robot parameters have to be tuned by hand per platform.

### 1.2 NRMP: solver migration (ECOS → SCS)

The QP was originally solved with ECOS. That blocked the whole fast path: the
differentiable-QP layer used for training does not support every solver, and ECOS in the
deployed loop was the bottleneck.

- Switched the deployed solver to **SCS**, which the canonicalisation layer supports.
- Kept the problem structure identical, so planner behaviour is unchanged apart from speed.

### 1.3 Fast path: skip derivative preparation + warm start

Because ONS2 plans in a receding horizon at a fixed rate, consecutive QPs are very similar.

- The derivative/canonicalisation work is **prepared once and reused** across ticks
  instead of being rebuilt each time.
- **SCS is warm-started** from the previous solution, so the solver starts near the answer.

### 1.4 Horizon and DUNE sample count

Both `receding` and the number of DUNE sample points were re-tuned against real runs.
Larger horizons look further ahead but cost more; the shipped default is the balance point
we validated on hardware. Both are exposed in the planner config — see *Where to tune*.

---

## 2. Behaviour quality (not just speed)

Faster is worthless if the robot drives worse. These changes came from field failures and
are all enabled in the shipped profiles.

### 2.1 Corridor-adaptive obstacle distance (`adaptive_d_max`)

A single fixed obstacle-influence radius is wrong for both environments: too small in open
space, too large in narrow corridors. The planner now measures the local corridor width
and adapts the influence distance, with a guard that falls back to a conservative value
when the estimate is unreliable.

Result in field tests: **noticeably faster progress through corridors** and a cheaper
solve, with no added collisions.

### 2.2 Narrow-corridor deadlock fix

In a corridor whose clear width left almost no margin per side, the planner would fall into
repeated reversing. The root cause was **not** the reversing logic: the obstacle influence
distance exceeded the available side clearance, so every tick the planner saw the walls as
unavoidable. Fixing the adaptive distance (2.1) removed the trigger; the reversing logic was
left untouched.

### 2.3 Rotation clearance gate

When the robot must rotate in place (alignment or re-planning), the planner checks the
**geometric footprint circle** against the current scan before rotating. In tight spaces it
backs off instead of turning into a wall corner. This measurably reduced oscillation in
doorways and narrow corridors.

### 2.4 Wall-clearance cost in global planning

The global planner's Theta\* cost includes a soft penalty for hugging obstacles, which
reduces corner scraping and gives the local planner a cleaner reference path to track.

---

## 3. System level

| Change | Why |
|---|---|
| **CPU affinity** | Pinning the planner to dedicated cores keeps its latency floor stable instead of competing with perception |
| **Power mode** | Raising the Jetson power mode keeps clocks from throttling mid-mission |
| **Real/sim strategy parity** | Strategies and adaptive behaviour are shared between simulation and robot, so tuning done in sim transfers instead of silently diverging |

---

## Where to tune

Planner settings live in `NeuPAN/neupan/ros/configs/`:

| File | Contents |
|---|---|
| `planner*.yaml` | Robot footprint, horizon (`receding`), DUNE sample count, limits, costs |
| `config*.yaml` | Adaptive behaviour, rotation clearance, latency compensation, debug switches |

Profiles are selected per platform, so you can change speed/clearance behaviour without
touching code. See `README_档位说明.md` in the same directory for the profile semantics.

> **A note on tuning order.** Optimising the solver first and behaviour second is the
> wrong order — a planner that is fast but unstable is worse than a slow but predictable
> one. Change one gate at a time, keep the shipped defaults as the baseline, and re-run the
> same scenario before accepting a change.

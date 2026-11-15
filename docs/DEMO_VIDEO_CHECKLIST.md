# Demo video (Deliverable 2)

Video link for the report and slides: **[INSERT_YOUTUBE_VIDEO_URL]** (upload as *Unlisted* unless told otherwise).

The spec asks for "a video demonstrating the robot detecting the tag and moving towards it". The video below does
that, and in 80 s also shows the method (predict + correct), why the correction matters, and the 10-trial result.
Everything except the phone footage is rendered from the logged data (`steps.csv`, camera frames), synced to the
phone video, so it looks live but is exact.

## Storyboard (1920 × 1080, about 80 s)

| # | Time | Scene | On-screen caption (lower third) |
|---|---|---|---|
| 1 | 0–5 s | **Title card.** RoboMaster EP, project name, one-line idea | *Phase 3 · RoboMaster Odometry*<br>*Predict with wheel odometry, correct with the camera (5-state EKF)* |
| 2 | 5–12 s | **Method card.** Animated loop: `/odom (v, ω)` → **Predict** → `x̂, P` → **Correct** with tag `(d, φ)` → `x̂⁺, P⁺`, each block lights up in turn | *Start 1.20 m · stop 0.50 m · same start for every trial* |
| 3 | 12–42 s | **Live trial dashboard** (layout below). Last 2 s before the stop at half speed | Changes with the robot state, see below |
| 4 | 42–57 s | **"Why correct?"** Trial 6 replayed on the top view: wheels-only path (green) drifts, EKF (blue) stays on target. Ends on two bars | */odom glitched 4 times. Wheels alone: +5.8 cm. EKF: −0.4 cm* |
| 5 | 57–75 s | **Results.** The 10 trials drop onto the scatter one by one, then three numbers | *10 trials · error = estimated − measured*<br>**MAE 0.28 cm · range 0.96 cm · wheels only 1.98 cm** |
| 6 | 75–80 s | **End card.** GitHub link, name, course | — |

### Scene 3 layout (live trial)
```
┌─ SCE-672 Phase 3 · EKF odometry ─────────────── DRIVING ── t = 6.4 s ─┐
│                                   │                                    │
│   PHONE (real robot)              │   TOP VIEW                         │
│                                   │   tag, robot box + heading arrow   │
│                                   │   EKF path (blue), wheels (green)  │
│                                   │   camera cone, 2σ ellipse          │
├──────────────────┬────────────────┴───────┬────────────────────────────┤
│ ROBOT'S EYE      │ DISTANCE TO TAG vs TIME │ TERMINAL (log lines)       │
│ camera + green   │ EKF ±2σ band, camera    │                            │
│ box on the tag   │ dots, wheels-only line  │                            │
│                  │  d̂ = 0.73 ± 0.01 m      │                            │
└──────────────────┴─────────────────────────┴────────────────────────────┘
```
Only three numbers are on screen: **d̂ ± σ**, the **camera distance**, and **speed**. Each camera fix makes a
short flash on the top view and the ellipse visibly shrinks; between fixes it grows (prediction).

Scene 3 captions, triggered by the logged state:

| When | Caption |
|---|---|
| WAITING | *1 · Detect: tag ID 3 found, 10 camera fixes before moving* |
| DRIVING, first 2 s | *2 · Predict: wheel odometry moves the estimate, uncertainty grows* |
| DRIVING, middle | *3 · Correct: each camera fix (range + bearing) pulls it back, uncertainty shrinks* |
| DRIVING, slowing | *Speed ∝ distance left · steering keeps the tag centred* |
| HOLDING | *4 · Stop when the EKF says 0.50 m · 2 s settle* |
| DONE + tape shot | *Tape: 0.4xx m → error = estimated − measured = x mm* |

## How to film the phone part

**Phone settings**
- Landscape, 1080p, 30 or 60 fps. Clean the lens.
- Tap and hold on the robot to lock focus and exposure (AE/AF lock), so the brightness does not pump.
- No zoom (digital zoom looks soft).

**Placement**
- Tripod (or a stack of books) about **1.2–1.5 m high**, at roughly **45° from the side**, looking down at the path.
- In frame: the robot on the start line, the whole path, and the tag. Leave a little space around them.
- Put the tape measure on the floor along the path, and two masking-tape labels: **START 1.20** and **STOP 0.50**.
- All room lights on, no shadow on the tag, nobody walking through the frame.

**Takes (three short files)**
1. `setup.mp4`, 4 s: slow move along the setup (tag → start line → robot). Handheld is fine.
2. `run.mp4`, the main take, tripod, do not touch the phone:
   start recording → wait 3 s → tell Claude **"film"** → robot waits for the tag, drives, stops →
   keep recording 3 s after it stops → stop.
3. `tape.mp4`, 4–5 s, handheld close-up: tape from the middle of the chassis front to the tag centre, hold
   still on the reading for 2 s. Send the reading to Claude.

Copy the three files to `~/Videos/phase3/`. No screen recording is needed: the terminal, robot's-eye view and plots
are rendered from the run's logs. Sync is automatic (the moment the robot starts moving in the phone video is matched
to `WAITING -> DRIVING` in the log).

## Upload
YouTube, *Unlisted*, title "SCE-672 Phase 3: RoboMaster odometry with an EKF (prediction + correction)".
Put the link at the top of this file, in the README and in the report.

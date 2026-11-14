# Code walkthrough (Deliverable 3)

A simple, file-by-file explanation to keep open while presenting the repo. For each file: **the idea**
in plain words, **what's inside**, and **one line to say**. For the exact formula on each line, keep
[`theory/FORMULA_MAP.md`](../theory/FORMULA_MAP.md) open in a second tab.

---

## 0. The whole system in one picture

Think of walking toward a poster with your eyes half closed:

- You **count your steps**: "5 steps, so about 1 m." That's **odometry** (`/odom`). It's smooth, but if your
  feet slip the count is wrong, and the mistake keeps adding up.
- Now and then you **peek**: "the poster is 0.8 m away, a little to the left." That's the **camera + tag**.
  It fixes the step-counting mistakes.
- A **judge** listens to both and decides how much to trust each one. That's the **EKF**. It also keeps a
  "how unsure am I" number, the covariance **P**. P grows while you only count steps and shrinks each time you peek.
- A **driver** keeps the robot straight and brakes when the judge says 40 cm is left. That's the **controller**.
- A **notebook** writes everything down 20 times a second. That's the **trial recorder**.

Every tick: **predict → check /odom → check camera → drive or brake → write it down.**

```
 camera ──► tag_detector.py ──(range, bearing)──┐
                                                ▼
 /odom ──(speed, turn rate)──────────► phase3_node.py ──► ekf.py (predict + correct)
                                                │                 │
                                                ▼                 ▼ distance to tag
                                         controller.py ◄──────────┘
                                                │
                                 /cmd_vel ◄─────┘      trial_logger.py ──► results/trials/…
```

---

## 1. `ekf.py`: the filter

**Idea:** a notebook with 5 numbers about the robot, plus a matrix that says how sure we are about each one.

| Name in code | Plain meaning | Symbol in the lectures |
|---|---|---|
| `state` | the 5 numbers: x, y, heading, speed, turn rate | $x=[p_x,p_y,\theta,v,\omega]$ |
| `cov` | how unsure we are, and how the numbers are linked | $P$ |
| `tag_position` | where the tag is nailed down (fixed, not estimated) | $(m_x,m_y)$ |
| `accel_noise`, `slip_noise` | how much the motion can surprise us | $Q$ |
| `odom_noise`, `tag_noise` | how noisy each sensor is | $R$ |
| `outlier_limit` | cut-off for readings that make no sense | χ² gate |
| `innovation` | sensor minus prediction | $e$ |
| `innovation_cov` | how big that difference normally is | $S$ |
| `gain` | how much to believe the sensor | $K$ |

**Walking through it**
- **L18–19** `STATE_SIZE`, `IDX_X … IDX_TURN_RATE`: names for the 5 slots, so the code says `state[IDX_SPEED]`
  instead of `state[3]`.
- **L22–24 `wrap_angle`**: like a clock face. Go past 180° and you come back at −179°. Without it, +179° and
  −179° look 358° apart instead of 2°, and the robot would "correct" itself into a spin (Lec 12B).
- **L29–39 `predict_state`**: "if I move at this speed for dt, where do I end up?" x grows by
  `speed·cos(heading)·dt`, y by `speed·sin(heading)·dt`, heading by `turn_rate·dt`. Speed and turn rate change
  by the accelerations.
- **L42–56 `motion_jacobian`** ($F$): if my guess is slightly off now, how off will it be after the step? The
  `cos·dt` entry is where a speed mistake turns into a position mistake.
- **L59–64 `input_jacobian`** ($G$): an unexpected push only changes speed and turn rate directly.
- **L69–73 `ODOM_JACOBIAN`** ($H_{odom}$): `/odom` only reports speed and turn rate, so this matrix just picks
  those two slots. **L76–78 `expected_odom`**: what `/odom` should say.
- **L81–87 `expected_tag_reading`**: "if I'm really here, how far is the tag and at what angle?" Range by
  Pythagoras, bearing = direction to the tag minus where the nose points.
- **L90–103 `tag_jacobian`** ($H_{tag}$): how range and bearing change if I move a little. Range cares about
  forward/back; bearing cares about left/right and heading.
- **L108 `RobotEKF`**
  - **L129–138 `predict`**: eyes closed, take a step. Move the state, keep the heading on the clock face, and
    grow P: `F·P·Fᵀ + G·Q·Gᵀ + slip·dt`.
  - **L140–157 `_correct`**: eyes open, fix the guess. The same five steps for both sensors:
    1. L142 innovation = measured − expected (L144 wraps it, **only** for bearings)
    2. L146 `S = H P Hᵀ + R`
    3. L148–150 if the reading is way off (NIS above the limit), skip it
    4. L152 `K = P Hᵀ S⁻¹`: very unsure → believe the sensor a lot; noisy sensor → believe it a little
    5. L153 move the state by `K·e`, L155 shrink P with `(I − K H) P`, L156 keep P symmetric
  - **L159–161 `correct_with_odom`**: correct using `/odom`. No `/imu` on top, since `/odom` already includes it.
  - **L163–171 `correct_with_tag`**: correct using the camera. Range and bearing, or bearing only for the comparison run.
  - **L174–186 `distance_from_front`**: the number we report. Centre-to-tag distance minus centre-to-front,
    so it matches the tape. Also returns σ from P.

**Say:** "ekf.py is only maths, no ROS, so I can test every formula on my laptop. Predict grows P, each sensor
shrinks it, and the gain K decides how much each sensor is trusted."

---

## 2. `controller.py`: the driver

**Idea:** a careful driver with four modes: WAITING (don't move until the tag has been seen 10 times),
DRIVING, HOLDING (stopped, wait 2 s), DONE.

- **L19 `DriveSettings`**: the rules: stop at 0.40 m, top speed 0.15 m/s, gentle acceleration, safety limits.
- **L54 `update`**: called every tick, returns `(speed, turn_rate)`.
  - Target speed = `slowdown_gain × distance still to go`, kept between min and max speed.
  - **L81** ramps toward that target gently, so the wheels don't slip at the start.
  - **L82** steers with the filtered bearing, so steering still works when the camera blinks.
- **L99 `_why_stop`**: the normal stop is "filtered distance ≤ 0.40 m". ToF, distance travelled and a timer are
  only safety stops.
- **L89 `emergency_stop`**: used by the node if the camera arm moves.
- **L112 `bearing_to`**: angle from the nose to the tag, wrapped.

**Say:** "The robot brakes on the filtered distance, not the raw camera, so both prediction and correction
decide where it stops."

---

## 3. `tag_detector.py`: the camera (from Phase 2)

**Idea:** look at each image, find tag ID 3, and say how far away it is and how far to the left.

- **L86 `process`**: keeps only the newest image → grayscale → pupil-apriltags gives the tag position →
  `camera_to_level` removes the camera's downward tilt → `front_range = scale·forward + offset` (from the tape
  calibration) → `bearing = atan2(left, forward)` → published on `tag/range_bearing`.
- **L68 `image_watchdog`**: if images stop for 2 s (Wi-Fi), subscribe again.
- **L116 `publish_debug`**: draws the green box you show in the video.
- `geometry.py`: the tilt maths from Phase 2.

**Say:** "The camera gives range and bearing. Range matters most, because it's the only sensor that sees
along-track drift."

---

## 4. `phase3_node.py`: connects everything

- **L40 `ApproachTagNode.__init__`**: reads `robot.yaml`, puts the tag at `(start_distance + front_offset, 0)`
  (L91), and builds the filter, controller and recorder. It subscribes to `odom`, `tag/range_bearing`, the ToF
  and the arm. **It never subscribes to `/imu`.**
- **L153–172 `on_odom`**: speed and turn rate from `/odom`, sign-flipped for this robot. L171 also keeps a
  running "wheels only" distance for comparison.
- **L174 `on_tag`**: keeps a camera reading only if the detection quality is good.
- **L189 `tick`**, every 1/20 s, with numbered comments:
  1. **L198–204** predict (the lag model: acceleration = (command − current speed) / lag)
  2. **L206–212** correct with `/odom` if a new message came in
  3. **L214–231** correct with the camera; L219 adds `front_offset` because the model measures from the centre
  4. **L233–260** control and send `cmd_vel`
  5. **L262–279** write a log row and print a status line every 0.5 s
- **L285 `send`**: `linear.x = drive_sign × speed` (−1 on this robot).
- **L292 `finish`**: sends stop 5 times, then writes `summary.json` with the final distance, σ, P, the last K,
  and the camera-only and odometry-only finals.
- **L319 `shutdown`** / **L326 `main`**: the robot always gets a zero command, even on Ctrl+C.

**Say:** "Each tick is predict, correct with odometry, correct with the camera, control, log: the
prediction–correction loop from the lectures."

---

## 5. `trial_logger.py`: the notebook

- **L12 `STEP_COLUMNS`**: time, commands, raw sensors, the 5 states, the full 5×5 P (`P00…P44`), the 5×2 K
  (`K00…K41`), `d_est`, `sigma_d`, `d_odom_only`.
- **L26** makes `results/trials/trial_NN_<date_time>/`, **L36 `write_step`** writes one row,
  **L48 `write_summary`** writes `summary.json` (`d_meas`/`error` stay empty until you tape-measure).

---

## 6. Config and launch
- `config/robot.yaml`: every number lives here. `MEASURE` = measure on the robot, `P2` = from Phase 2.
- `launch/phase3.launch.py`: starts the detector and the node with `trial:=N`.

---

## 7. Scripts

| Script | What it does |
|---|---|
| `add_measurement.py N 0.412` | stores the tape distance; `error = d_est − d_meas`, plus camera-only and odometry-only errors |
| `analyze_trials.py` | table, statistics and 5 figures from all trials |
| `calibrate_range.py` | camera vs tape at several distances → `range_scale`, `range_offset` |
| `formula_lines.py` | prints the current line of every formula |

`analyze_trials.py`: `stats_of` computes MAE, range, bias, variance (n−1), σ and RMSE. `compute` adds covariance
and correlation between estimated and measured distance, and checks whether the real spread matches the filter's
own σ. The figures:
1. estimated vs measured
2. error per trial
3. distance vs time
4. σ vs time
5. statistics: EKF vs camera-only vs odometry-only

---

## 8. Tests (`src/rm_odom_ekf/test/`, 23 tests)
- Jacobians vs finite differences (Lec 11A slide 18)
- the wrap rules, and that range is never wrapped
- corrections shrink P
- odometry moves x through the cross term in P
- bearing alone can't fix along-track error
- outlier rejection
- the camera pulls back 8 % wheel slip
- controller modes and safety stops
- recorder files

---

## 9. Likely questions
1. **Which measurements correct the estimate?** `/odom` speed and turn rate, and camera range and bearing. The ToF is only a safety stop.
2. **Why no IMU?** `/odom` already contains it. Adding `/imu` counts it twice and makes P overconfident (Lec 11B slide 24).
3. **Why range and not just bearing?** Facing the tag, the bearing row of $H_{tag}$ has zero in the $p_x$ column (Lec 12A slide 26).
4. **How do you know the Jacobians are right?** The finite-difference tests.
5. **Is the filter honest?** Compare the spread of the real errors with the filter's σ (consistency ratio in the table).

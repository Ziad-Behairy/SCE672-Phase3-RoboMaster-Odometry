# Code Explanation Guide: the whole project, explained like you're in primary school (Deliverable 3)

Use this while you scroll through the repo live. Every file has the same parts:
**🧸 The story** (a simple picture), **🔧 What's inside** (each function), and **🎤 Say this** (one or two sentences
to say out loud). For the exact formula on each line, keep [`theory/FORMULA_MAP.md`](../theory/FORMULA_MAP.md)
open in a second tab.

---

## 0. The big picture (say this first)

🧸 Imagine walking toward a poster on the wall **with your eyes half closed**.

- 🦶 You **count your steps**: "5 steps, so about 1 metre." That's **odometry** (`/odom`). Quick and smooth,
  but if your feet slip the count is wrong, and the mistake keeps growing.
- 👀 Now and then you **peek**: "The poster looks this big, so it's 0.8 m away, a bit to my left." That's
  the **camera + tag**. It fixes your step-counting mistakes.
- ⚖️ A **judge** listens to both and decides who to trust more. That's the **Extended Kalman Filter (EKF)**.
  It also keeps a **"how unsure am I" bubble**, called **P**. The bubble grows while you only count steps and
  shrinks every time you peek.
- 🚗 A **driver** pushes the robot straight and hits the brake when the judge says "40 cm left." That's the
  **controller**.
- 📒 A **diary** writes down everything, 20 times a second. That's the **logger**.

Every 1/20 of a second the robot does: **guess → check feet → peek → drive or brake → write in the diary.**

```
 camera ──► tag_detector.py ──(d, φ)──┐
                                      ▼
 /odom ──(v, ω)──────────────► phase3_node.py ──► ekf.py (guess + correct)
                                      │                 │
                                      ▼                 ▼ distance to tag
                               controller.py ◄──────────┘
                                      │
                       /cmd_vel ◄─────┘        trial_logger.py ──► results/trials/…
```

---

## 1. `src/rm_odom_ekf/rm_odom_ekf/ekf.py`: the judge 🧠

🧸 A notebook with **5 numbers** about the robot and a **bubble** that says how sure we are about them.

| Variable | Kid version | Real meaning |
|---|---|---|
| `x` | "my 5 guesses" | $[p_x, p_y, \theta, v, \omega]$: where I am (x, y), which way my nose points, how fast I go, how fast I turn |
| `P` | "my worry bubble" (5×5) | covariance: how unsure each guess is, and how the guesses are linked |
| `tag_xy` | "where the poster is nailed" | known tag position $(m_x, m_y)$. It never moves, so it's not something we guess |
| `Q_u`, `q_pose` | "how wobbly my legs are" | process noise: things the motion model can't predict (slip, bumps) |
| `R_odom`, `R_tag` | "how blurry each sensor is" | measurement noise of `/odom` and the camera |
| `gate` | "the nonsense filter" | rejects camera readings that make no sense (9.21 = 99 % χ² limit) |

### Line by line
- **L15 `PX, PY, TH, V, W = range(5)`**: gives the 5 boxes names (0..4), so we write `x[V]` instead of `x[3]`.
- **L19–21 `wrap(angle)`**: 🧸 a clock face. Go past 12 and you're back at 1, not at 13. Angles past 180° come back
  as −179°. Uses `atan2(sin, cos)`. *Why:* +179° and −179° are only 2° apart; without wrap the robot thinks
  they're 358° apart and spins (Lecture 12B).
- **L26–36 `motion_model(x, u, dt)`**: 🧸 "If I walk at this speed for this long, where will I be?"
  - L31: x moves by `v·cos θ·dt` (forward part along the room's x)
  - L32: y moves by `v·sin θ·dt` (sideways part)
  - L33: heading turns by `ω·dt`
  - L34–35: speed and turn rate change by the accelerations `a·dt`, `α·dt`
- **L39–48 `jacobian_F`**: 🧸 "If my guess is a tiny bit wrong now, how wrong will it be after the step?"
  The `cos θ·dt` box says a speed mistake becomes a position mistake. That's how the worry spreads.
- **L51–59 `jacobian_G`**: 🧸 "a surprise push changes only my speed and turn rate."
- **L64–68 `H_ODOM`**: 🧸 `/odom` only tells us speed and turn rate, so this matrix just *points at* boxes 4 and 5.
- **L71–73 `h_odom(x)`**: what `/odom` *should* say if our guess were right.
- **L76–82 `h_tag(x, tag_xy)`**: 🧸 "If I'm really here, how far is the poster and at what angle?"
  `d = √(Δx² + Δy²)` (Pythagoras), `φ = atan2(Δy, Δx) − θ` (the poster's angle minus where my nose points).
- **L85–94 `jacobian_H_tag`**: how the poster's distance and angle change when I move a tiny bit.
  Row 1 (distance) cares about **forward/back** → fixes along-track drift. Row 2 (angle) cares about
  **left/right and nose direction**.
- **L99 `class EKF`**: the judge itself.
  - **L108 `__init__`**: write down the starting guesses `x0`, the starting bubble `P0`, and all the noise sizes.
  - **L118–124 `predict(u, dt)`**: 🧸 *close eyes and take a step.* Move the guess with `motion_model`, keep the
    angle on the clock with `wrap`, and **blow up the bubble**: `P = F P Fᵀ + G Q Gᵀ + Q_pose·dt`.
  - **L126–141 `_correct(...)`**: 🧸 *open eyes and fix the guess.* The five classic steps:
    1. L128 `e = z − h(x)`: "how much does the sensor disagree with me?" (L130: wrap it if it's an angle,
       **never** if it's a distance)
    2. L131 `S = H P Hᵀ + R`: "how big should that disagreement normally be?" (my worry + sensor blur)
    3. L133–135: if the disagreement is crazy big (NIS > gate), ignore this reading
    4. L136 `K = P Hᵀ S⁻¹`: **Kalman gain**. 🧸 "How much do I believe the sensor?" Very worried → believe a lot.
       Sensor very blurry → believe a little.
    5. L137 `x = x + K e`: move my guess part of the way toward the sensor
    6. L139 `P = (I − K H) P`: **shrink the bubble**, since I just learned something. L140 makes `P` perfectly symmetric.
  - **L143–145 `update_odom(v, w)`**: correct with `/odom`. The docstring says it: **no `/imu` on top**.
  - **L147–154 `update_tag(d, phi, use_range)`**: correct with the camera. Both rows normally, or bearing only
    for the comparison experiment.
  - **L156–167 `distance_to_tag(front_offset)`**: the number we report. Distance from robot centre to tag
    **minus** centre→front, so it's measured from the **chassis front**, just like the tape. Also returns
    σ (how sure), from the bubble: `σ = √(J P Jᵀ)`.

🎤 *"ekf.py is pure maths with no ROS, so I can test every formula on my laptop. Predict grows the uncertainty P,
each sensor update shrinks it, and the Kalman gain decides how much to trust each sensor."*

---

## 2. `controller.py`: the driver 🚗

🧸 A careful driver with four moods: **WAITING** (don't move until I've clearly seen the poster 10 times),
**DRIVING** (go, slow down near the end), **HOLDING** (stopped, take a breath for 2 s), **DONE**.

- **`ControllerParams`**: the driving rules: stop at 0.40 m, top speed 0.15 m/s, speed-up limit 0.25 m/s²,
  and the safety limits.
- **`step(...)`**: called 20× per second, returns `(v, ω)`.
  - WAITING → DRIVING when the EKF has `tag_settle` good camera fixes.
  - DRIVING: `v_target = k_v × (distance left)`, kept between `v_min` and `v_max`. The **ramp** line changes speed
    gently (no wheel slip). Steering `ω = k_heading × bearing` keeps the nose on the tag, using the **EKF's** bearing,
    so it still works if the camera blinks.
  - `_brake_reason`: brake when **EKF distance ≤ stop distance** (the normal stop). Safety brakes: ToF sensor
    too close, travelled too far, or too much time.
  - HOLDING for `hold_time` → DONE. The final estimate is read **after** the robot has stopped rolling.
- **`abort(...)`**: emergency brake used by the node (e.g. the camera arm moved).
- **`bearing_error(...)`**: angle from the nose to the tag, wrapped.

🎤 *"The controller brakes on the filtered estimate, not on the raw camera, so prediction and correction decide
when we stop. The ToF sensor is only a safety net."*

---

## 3. `tag_detector.py`: the eyes 👀 (reused from Phase 2)

🧸 Looks at each photo, finds the black-and-white square with the right number on it (ID 3), and says
"it's **this far** and **this many degrees** to the left."

- `__init__`: reads settings (tag size, camera tilt, calibration `range_scale`/`range_offset`).
- `subscribe_camera` + `image_watchdog`: if the Wi-Fi hiccups and images stop for 2 s, plug the camera back in.
- `on_info`: learns the camera's lens numbers (`fx, fy, cx, cy`) once.
- `on_image`: keeps only the newest photo (old photos are useless).
- **`process`**: the main job, 15× per second:
  1. turn the photo grey → `Detector.detect` finds tags and their 3-D position
  2. `camera_to_level`: undo the camera's downward tilt → `(forward, left, up)`
  3. `d = range_scale × forward + range_offset` → distance from the **chassis front** (calibrated with a tape)
  4. `φ = atan2(left, forward)` → angle, + means left
  5. publish `[d, φ, margin, stamp]` on `tag/range_bearing`
- `publish_debug`: draws green boxes on the image (for the demo video).

🎤 *"The camera gives two measurements, range and bearing. Range is the important one, because it's the only
sensor that can see along-track odometry drift."*

`geometry.py` is the Phase 2 helper with the tilt maths (`camera_to_level`).

---

## 4. `phase3_node.py`: the boss who connects everyone 🧑‍✈️

🧸 Listens to the eyes and the feet, asks the judge, tells the driver, and writes in the diary.

- **`__init__`**: reads every number from `config/robot.yaml`, puts the tag at
  `(start_distance + front_offset, 0)` (the robot centre starts at (0, 0), nose along +x), creates the
  EKF, controller and logger, and subscribes to `odom`, `tag/range_bearing`, `range_0` (ToF) and `arm_position`.
  **No `/imu`!**
- **`on_odom`**: reads `v` and `ω` from `/odom` (flipping the sign because this robot's driver x points backwards),
  and also adds up an **odometry-only** distance for comparison.
- **`on_tag`**: keeps a camera reading only if the detector was confident (`decision_margin`).
- **`tick`**: the heartbeat, 20× per second (L172 on). The numbered comments match the story:
  1. **predict** (L181–186): `u = (v_cmd − v)/τ` says "the chassis is catching up with the command"
  2. **odometry correction** (L188–194)
  3. **camera correction** (L196–213): only fresh readings; adds `front_offset` so it's centre→tag like `h(x)`
  4. **control** (L215–241): arm check, then `controller.step`, then send `cmd_vel`
  5. **log** (L243–258): one diary row and a terminal line every 0.5 s (nice for the video)
- **`send`**: `cmd_vel.linear.x = drive_sign × v` (−1 on this robot).
- **`finish`**: brakes 5 times to be sure, then writes `summary.json` with `d_est_final`, `σ`, final `P`,
  last `K`, camera-only and odometry-only finals.
- **`stop` / `main`**: whatever happens (even Ctrl+C), the robot gets a **zero-speed** command.

🎤 *"Every tick is predict, correct with odometry, correct with the camera, control, log. Exactly the
prediction–correction loop from the lectures."*

---

## 5. `trial_logger.py`: the diary 📒

🧸 One folder per trial. Inside: a big table of every moment (`steps.csv`) and a report card (`summary.json`).

- `STEP_COLUMNS`: the column names: time, commands, sensor readings, the 5 guesses, the full 5×5 `P`
  (`P00…P44`), the 5×2 Kalman gain `K` (`K00…K41`), `d_est`, `sigma_d`, `d_odom_only`.
- `TrialLogger.__init__`: makes `results/trials/trial_03_<date_time>/`.
- `log_step`: writes one row.
- `finish`: writes `summary.json`. `d_meas` and `error` start empty (`null`) until you measure with the tape.

---

## 6. `config/robot.yaml` and `launch/phase3.launch.py`: the settings sheet and the start button

- `robot.yaml`: every number lives here, not in the code. `MEASURE` = measure on your robot,
  `P2` = taken from Phase 2. Start 1.50 m, stop 0.40 m, the same for all trials.
- `phase3.launch.py`: starts the eyes (`tag_detector`) and the boss (`phase3_ekf`) with `trial:=N`.
- `package.xml`, `setup.py`, `setup.cfg`: the ROS 2 package's ID card and install instructions.

---

## 7. The scripts 🧮

| Script | Kid version | What it does |
|---|---|---|
| `scripts/add_measurement.py 3 0.412` | "write the tape number in the report card" | stores `d_meas`, computes `error = d_est − d_meas` (+ camera-only and odometry-only errors) |
| `scripts/analyze_trials.py` | "the teacher who grades all the report cards" | reads all trials → table (`.csv/.md/.tex`), `statistics.json`, 5 figures |
| `scripts/calibrate_range.py` | "teaching the eyes to judge distance" | records camera vs tape at several distances, fits `range_scale`, `range_offset` |
| `scripts/simulate_trials.py` | "a practice robot in the computer" | fake trials using the real EKF and controller, to test everything on Windows (`results/_demo`, never real results) |
| `scripts/formula_lines.py` | "find every formula" | prints file:line of every `[EQ …]` tag |

### `analyze_trials.py` functions
- `load_trials`: picks the newest run of each trial number that has a tape measurement and is valid.
- `stats_of(e)`: **MAE** = average of |e| · **range** = max − min · **bias** = average e ·
  **variance** σ² (with n−1) · **std** σ · **RMSE**.
- `compute`: the above for the EKF, camera-only and odometry-only, plus **covariance and correlation** between
  estimated and measured distance, and the **consistency check**: is the real spread about the same as the EKF's
  own σ from `P`? (ratio ≈ 1 means `P` is honest.)
- `fig_scatter` (Fig. 1): estimated vs measured, one dot per trial, ±2σ bars from `P`, dashed perfect line.
- `fig_errors` (Fig. 2): error per trial, mean ± σ band, and the two one-sensor baselines.
- `fig_time` (Fig. 3): distance to the tag over time for every trial: EKF vs camera dots vs odometry-only.
- `fig_uncertainty` (Fig. 4): σ from `P` over time. The saw-tooth = grow while predicting, drop at each camera fix.
- `fig_breakdown` (Fig. 5): MAE / bias / σ / range / RMSE bars for EKF vs camera-only vs odometry-only.

---

## 8. The tests 🧪 (`src/rm_odom_ekf/test/`)

🧸 Little exams the code must pass before it's allowed on the robot. Run them with
`cd src/rm_odom_ekf && python3 -m pytest -q test` (23 tests).

- `test_F_/G_/H_tag_matches_central_differences`: checks every Jacobian with the "wiggle it a tiny bit"
  method from Lecture 11A slide 18 ($h = 10^{-6}$).
- `test_wrap_*`, `test_bearing_innovation_is_wrapped_across_pi`: the clock-face rule.
- `test_range_innovation_is_not_wrapped`: distances are **never** wrapped.
- `test_updates_shrink_uncertainty`: corrections make the bubble smaller and keep `P` valid.
- `test_odom_corrects_position_through_cross_covariance`: odometry moves `px` through `P_xv`.
- `test_bearing_only_barely_fixes_along_track`: the Lecture 12A slide 26 point, as a test.
- `test_gate_rejects_wild_tag_reading`, `test_simulated_run_keeps_final_distance_error_small`.
- `test_controller.py`: wait → drive → ramp → brake → hold → done, safety brakes, steering sign, diary files.

---

## 9. Five questions Dr. Bhatti might ask, and short answers

1. **"Which sensor measurements are used for correction?"** `/odom` ($v,\omega$) with $H_{odom}$, and the camera
   tag (range $d$ + bearing $\varphi$) with $H_{tag}$. The ToF is only a safety brake.
2. **"Why not fuse the IMU?"** `/odom` already contains it; fusing both counts the gyro twice and makes `P`
   overconfident (Lec 11B slide 24).
3. **"Why do you need range and not only bearing?"** Driving straight at the tag, the bearing row of $H$ has a
   zero in the $p_x$ column, so bearing can't see along-track drift (Lec 12A slide 26).
4. **"How do you know your Jacobians are right?"** Central-difference unit tests, `test_ekf.py`.
5. **"Is your filter honest?"** Compare the real spread of the errors with the EKF's σ (consistency ratio and
   "trials within 2σ" in the statistics table).

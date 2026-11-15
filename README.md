# SCE-672 Project Phase 3: RoboMaster Odometry

Ziad Aboelkhir, KFUPM, Advanced Robotics (SCE-672), Prof. Aamer Iqbal Bhatti. DJI RoboMaster EP Core, Ubuntu 24.04, ROS 2 Jazzy.

The robot starts **1.20 m** in front of a fixed tag (AprilTag tag36h11, ID 3), detects it, drives straight toward it
and stops **0.50 m** before it (both measured from the chassis front). Its distance to the tag is estimated
throughout by a **5-state Extended Kalman Filter**:

- **Prediction**: unicycle motion model with velocity states, $x=[p_x,p_y,\theta,v,\omega]^T$ (Lecture 11A)
- **Correction 1, odometry**: `/odom` speed and turn rate, $z=[v,\omega]$ (Lecture 11B). No separate `/imu`: `/odom` already contains it
- **Correction 2, camera**: tag range and bearing, $z=[d,\varphi]$, with the bearing innovation wrapped (Lectures 12A, 12B)

8–10 trials are compared with a tape measurement: $e = d_{est} - d_{meas}$, MAE, range, bias, σ, covariance.

**Code:** https://github.com/Ziad-Behairy/SCE672-Phase3-RoboMaster-Odometry · **Video:** [INSERT_YOUTUBE_VIDEO_URL]

## The 7 presentation deliverables

| # | Deliverable | Where |
|---|---|---|
| 1 | Public GitHub repo, clean folders | this repo · [docs/GITHUB_SETUP.md](docs/GITHUB_SETUP.md) |
| 2 | Demo video | [docs/DEMO_VIDEO_CHECKLIST.md](docs/DEMO_VIDEO_CHECKLIST.md) |
| 3 | Line-by-line code walkthrough | [docs/CODE_EXPLANATION_GUIDE.md](docs/CODE_EXPLANATION_GUIDE.md) |
| 4 | Formulas used, where and why | [theory/FORMULA_MAP.md](theory/FORMULA_MAP.md) (+ `python3 scripts/formula_lines.py`) |
| 5 | Role of odometry | [theory/ODOMETRY_ROLE.md](theory/ODOMETRY_ROLE.md) |
| 6 | Trial-by-trial scatter and error plots | `results/figures/fig1_*`, `fig2_*` from [scripts/analyze_trials.py](scripts/analyze_trials.py) |
| 7 | Full statistics | `results/summary_table.md`, `results/statistics.json`, `results/figures/fig5_*` |

## Layout
```
src/rm_odom_ekf/
  rm_odom_ekf/ekf.py            5-state EKF: f, F, G, H_odom, h(x), H_tag, K, P, wrap   (pure maths, no ROS)
  rm_odom_ekf/controller.py     WAITING -> DRIVING -> HOLDING -> DONE, brake at the EKF stop distance
  rm_odom_ekf/tag_detector.py   AprilTag range + bearing from the chassis front (from Phase 2)
  rm_odom_ekf/geometry.py       camera tilt helpers (from Phase 2)
  rm_odom_ekf/phase3_node.py    ROS 2 node: predict -> odom update -> tag update -> control -> log
  rm_odom_ekf/trial_logger.py   steps.csv (every tick, full P and K) + summary.json per trial
  config/robot.yaml             all parameters (start/stop distance, noise, calibration)
  launch/phase3.launch.py       ros2 launch rm_odom_ekf phase3.launch.py trial:=N
  test/                         23 unit tests incl. central-difference Jacobian checks
scripts/                        add_measurement, analyze_trials, calibrate_range, formula_lines
theory/                         theory_equations.md, FORMULA_MAP.md, ODOMETRY_ROLE.md, project spec
results/                        trials/, figures/, summary_table.*, statistics.json
docs/                           guides (code explanation, robot runbook, video, GitHub)
```

## Quick start
**Windows or Ubuntu, no robot:** unit tests for the filter and controller
```bash
python -m pip install numpy matplotlib pytest
cd src/rm_odom_ekf && python -m pytest -q test
```
**On the robot:** follow [docs/RUN_ON_ROBOT.md](docs/RUN_ON_ROBOT.md) (install, calibration, sign check, trials).

## Results (10 trials, start 1.20 m, stop 0.50 m)
| | EKF (predict + correct) | camera only | wheels only |
|---|---:|---:|---:|
| **MAE** | **0.28 cm** | 0.29 cm | 1.98 cm |
| **Range of errors** | **0.96 cm** (−0.48 … +0.48) | 0.95 cm | 6.31 cm |
| Bias / σ | −0.10 cm / 0.32 cm | | |

Full table: [results/summary_table.md](results/summary_table.md) · figures: [results/figures/](results/figures/) ·
sensor-removal replay (bearing only 1.98 cm, no camera 2.53 cm): [results/replay/replay_table.md](results/replay/replay_table.md) ·
report: [report/](report/) (Overleaf: `report/overleaf_upload.zip`) · video: `results/video/phase3_demo.mp4`.

Setup changes found during calibration (details in the report): start 1.20 m instead of 1.50 m (the flat tag is ~10 px
tall at 1.5 m), stop 0.50 m (the tag leaves the image below 0.47 m), `/odom` speed taken from the pose (the twist field
lags and over-reads), and a plausibility guard against `/odom` pose glitches.

Reused from Phase 2 ([advanced-robotics-project-2](https://github.com/Ziad-Behairy/advanced-robotics-project-2)):
the tag detector, camera geometry, robot calibration values and the safe-stop pattern.

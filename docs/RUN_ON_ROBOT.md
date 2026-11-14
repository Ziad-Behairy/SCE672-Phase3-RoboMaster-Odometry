# Running Phase 3 on the robot (Ubuntu 24.04, ROS 2 Jazzy)

Plan for two sessions: **session A** for setup and calibration (about 1.5 h), **session B** for the 10 trials (about 1 h).
Write every measurement in [TRIAL_SHEET.md](TRIAL_SHEET.md) as you go, with the time.

## What to bring
- Steel tape measure (mm marks) and a 30 cm ruler
- Masking tape and a marker for the floor
- Printed tag36h11 **ID 3** (`docs/apriltags_tag36h11_T1_T2_T3_A4.pdf`, page 3), stuck flat on cardboard
- Something to stand the tag upright: a box or a wall
- A box to lift the robot's wheels off the ground
- Phone and tripod for the video
- Fully charged battery

---

# Session A: setup and calibration

## A1. Install the code (Ubuntu)
```bash
cd ~ && git clone https://github.com/Ziad-Behairy/SCE672-Phase3-RoboMaster-Odometry SCE672-Phase3-RoboMaster-Odometry
ln -s ~/SCE672-Phase3-RoboMaster-Odometry/src/rm_odom_ekf ~/ros2_ws/src/rm_odom_ekf
cd ~/ros2_ws && colcon build --symlink-install --packages-select rm_odom_ekf
source ~/rm_setup.sh && source ~/ros2_ws/install/setup.bash
pip install pupil-apriltags                 # skip if Phase 2 still runs
cd ~/SCE672-Phase3-RoboMaster-Odometry/src/rm_odom_ekf && python3 -m pytest -q test     # expect: 23 passed
```
All the numbers you measure below go into `src/rm_odom_ekf/config/robot.yaml`. After editing it, rebuild with the
same `colcon build` line. With `--symlink-install` a rebuild takes a few seconds.

## A2. Measure the tag (`tag_size`)
1. Put the ruler on the **black square only**, edge to edge (not the white border).
2. Measure both sides and use the average, in metres (e.g. 155 mm → `0.155`).
3. Write it in `tag_size`. **Why it matters:** the camera works out distance from how big the tag looks. A 5 % size
   error gives a 5 % range error: 7.5 cm at 1.5 m.

## A3. Set up the scene (keep it identical for all trials)
1. Stand the tag **upright**, facing where the robot will start. Its **centre at the camera's height** (measure the
   camera lens height from the floor and match it).
2. Pick the **reference point** on the robot: the **middle of the front edge of the chassis** (camera side). Use this
   one point for every distance in the project.
3. From the **face of the tag**, tape **1.50 m** along the floor and mark a start line at right angles to that path.
4. Put the robot on the line so its reference point is on the mark and its nose points at the tag. Put tape at both
   front wheels so you can put it back exactly the same way every time. Photograph the setup.

## A4. Measure `front_offset` (robot centre → chassis front)
`/odom` and the filter track the **robot centre**, which is the midpoint between the four wheel axles.
1. Measure the distance between the front and rear axle centres (wheelbase), and halve it.
2. Measure from the front axle centre forward to the chassis front edge.
3. `front_offset` = half wheelbase + axle-to-front. Expect about 0.16 m. Write it to the nearest mm.

## A5. Set the camera arm
The camera calibration from Phase 2 holds only at one arm pose (`x 0.123, z 0.067`).
1. Start the driver (terminal 1):
   ```bash
   ros2 launch robomaster_ros ep.launch conn_type:=sta video_resolution:=720 chassis_timeout:=0.6 tof_0:=true
   ```
2. Terminal 2: `ros2 topic echo /arm_position --field point`
3. Set the arm by hand, about 2 cm above the ToF sensor as in Phase 2, until x ≈ 0.123 and z ≈ 0.067.
4. If you choose a different pose because the tag is upright now, write the x and z you got into
   `arm_x_saved`/`arm_z_saved`. Then **do A7 at that pose**.

## A6. Check the drive and `/odom` directions (2 minutes)
Clear path toward the tag. Driver running.
```bash
# terminal 2
ros2 topic echo /odom --field twist.twist
# terminal 3: creep toward the tag for ~2 s, then Ctrl+C
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05}}"
```
| What you see | What to set |
|---|---|
| robot moves **toward** the tag and `linear.x` is **negative** | keep `drive_sign: -1.0`, `odom_v_sign: -1.0` |
| robot moves toward the tag and `linear.x` is **positive** | `odom_v_sign: 1.0` |
| robot moves **away** from the tag | `drive_sign: 1.0`, then repeat the test with `x: 0.05` |
| `linear.x` and `angular.z` stay **0.0** while moving | `odom_source: pose` |

Then turn slowly to the left: `"{angular: {z: 0.3}}"`. `angular.z` should be **positive**. If it's negative, set
`odom_w_sign: -1.0`.

## A7. Calibrate the camera range (`range_scale`, `range_offset`)
1. Detector only (terminal 2):
   ```bash
   ros2 run rm_odom_ekf tag_detector --ros-args --params-file ~/SCE672-Phase3-RoboMaster-Odometry/src/rm_odom_ekf/config/robot.yaml
   ```
2. Put the robot facing the tag with the reference point at a taped distance. At each of **0.40, 0.70, 1.00, 1.30,
   1.60 m**, keep the robot still and run (terminal 3):
   ```bash
   cd ~/SCE672-Phase3-RoboMaster-Odometry
   python3 scripts/calibrate_range.py --tape 0.40
   ```
   Each command records 5 s and appends one row to `results/calibration/tag_range.csv`.
   If it says NOT DETECTED, check the lighting and the tag size.
3. Fit and copy the result into `robot.yaml`:
   ```bash
   python3 scripts/calibrate_range.py --fit
   ```
   Residuals should be within **±1 cm**. If one point is way off, redo that distance.
4. Rebuild. Spot check: put the robot at 1.20 m, run `ros2 topic echo /tag/range_bearing`, and the first number
   should read 1.20 ± 0.01.

## A8. Dry run, wheels off the ground
Robot on the box. Driver running.
```bash
ros2 launch rm_odom_ekf phase3.launch.py trial:=0
```
Check:
1. "Camera arm in place", then `WAITING -> DRIVING`
2. the wheels turn the right way
3. `d_est` goes down in the printed lines
4. it ends with `Trial 0 done`

Then `rm -rf results/trials/trial_00_*`.
On the box the camera still sees the tag at the same distance, so expect the run to end on the safety limits.
That's fine; this is only to check the wiring.

## A9. One test drive on the floor
Run one real drive with `trial:=0` and check the robot stops about 0.40 m from the tag and doesn't swerve. Delete it.
Commit the calibration:
```bash
git add src/rm_odom_ekf/config/robot.yaml results/calibration && git commit -m "Calibration for Phase 3" && git push
```

---

# Session B: the 10 trials

Same room, floor, lighting and tag position as session A. Battery above 50 %. Don't change `robot.yaml` between trials.

For **N = 1 … 10**:
1. **Place** the robot on the start marks, nose at the tag, arm at the saved pose. Don't touch it after this.
2. **Run** (driver already running in terminal 1):
   ```bash
   ros2 launch rm_odom_ekf phase3.launch.py trial:=N
   ```
3. **Wait** until the terminal prints `Trial N done (...): d_est = 0.xxx ± 0.0xx m`. Copy `d_est`, `±`
   and the stop reason into the trial sheet.
4. **Measure** with the tape, without moving the robot: from the **reference point** (chassis front middle)
   straight to the **tag face**, along the floor, read to the **millimetre**. Measure twice and use the average.
   Write it on the sheet.
5. **Log** it:
   ```bash
   python3 scripts/add_measurement.py N 0.412
   ```
   It prints the error. Write it on the sheet too.
6. **Notes**: anything odd (wheel slip noise, the tag lost for a moment, a bump, somebody walking past).
7. If the stop reason ends with `(safety)`, the trial is marked invalid. Run it again with the **same N**; the
   analysis uses the newest valid run.

After trial 10:
```bash
python3 scripts/analyze_trials.py
git add results && git commit -m "Phase 3 trials 1-10" && git push
```
This writes `results/summary_table.md` (table + statistics), `results/statistics.json` and `results/figures/`
(Fig. 1 estimated vs measured, Fig. 2 error per trial, Fig. 3 distance vs time, Fig. 4 σ vs time, Fig. 5 statistics).

## Optional: bearing-only runs for Section 4 of the report
1. Set `use_tag_range: false` in `robot.yaml` and rebuild.
2. Run two or three trials:
   ```bash
   ros2 launch rm_odom_ekf phase3.launch.py trial:=1 results_dir:=~/SCE672-Phase3-RoboMaster-Odometry/results/bearing_only
   ```
3. Measure each one the same way, with `python3 scripts/add_measurement.py 1 0.4xx --trials-dir results/bearing_only`.
4. Analyse them:
   ```bash
   python3 scripts/analyze_trials.py --trials results/bearing_only --out results/bearing_only --label "(bearing only)"
   ```
5. Set `use_tag_range` back to `true`.

The error should be clearly bigger than in the main trials, which shows that range is what fixes along-track drift.

## If something goes wrong
| Symptom | Likely cause / fix |
|---|---|
| Stays in WAITING, "tag not seen" | Tag too small/far or poorly lit. Check `rqt_image_view /tag/debug_image` for the green box. |
| "Waiting for the saved camera arm pose" | The arm isn't at the saved pose (A5). |
| Robot drives away from the tag | `drive_sign` (A6) |
| `d_est` goes **up** while driving | `odom_v_sign` (A6) |
| Many `tag_rejected` in summary.json | Range calibration off (A7) or wrong `tag_size` (A2) |
| Stops early with `ToF ... (safety)` | Something in front of the ToF sensor, or the tag stand is too close to the path |
| Driver disconnects (Wi-Fi) | Use router (STA) mode as in Phase 2 |

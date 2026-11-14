# Running Phase 3 on the robot (Ubuntu 24.04, ROS 2 Jazzy)

Do steps 1–6 once. Steps 7–8 are the trials. Keep a notebook (or `results/lab_log.md`) with the time and value
of everything you measure.

## 1. Install the package
```bash
cd ~ && git clone [INSERT_GITHUB_REPO_URL] SCE672-Phase3-RoboMaster-Odometry
ln -s ~/SCE672-Phase3-RoboMaster-Odometry/src/rm_odom_ekf ~/ros2_ws/src/rm_odom_ekf
cd ~/ros2_ws && colcon build --symlink-install --packages-select rm_odom_ekf
source ~/rm_setup.sh && source ~/ros2_ws/install/setup.bash
pip install pupil-apriltags            # already installed if Phase 2 still runs
cd ~/SCE672-Phase3-RoboMaster-Odometry/src/rm_odom_ekf && python3 -m pytest -q test   # 23 passed
```

## 2. The scene (identical for every trial)
- **Tag**: tag36h11 **ID 3** (`docs/apriltags_tag36h11_T1_T2_T3_A4.pdf`). Print it as large as you can (A3 with a
  ~250 mm black edge is best; on A4 the edge is ~155–180 mm). **Measure the black edge with a ruler** and put it in
  `tag_size`. Phase 2's floor tag was only reliable to ~1.2 m because it was seen almost edge-on. Standing the
  tag **upright, facing the robot, its centre at camera height** makes it much easier to detect at 1.5 m.
- **Floor marks**: tape a start line **1.50 m** from the tag face, plus two marks for the robot's side edges so it's
  placed the same way every time.
- **Reference point**: the **middle of the chassis front edge** (camera side). The same point for the start
  distance, the EKF output, and the tape at the end.
- Same floor, lighting and battery level (> 50 %) for all trials.

## 3. Measure `front_offset`
Distance from the robot centre (midpoint between the four wheel axles = `base_link` / `/odom` point) to the chassis
front edge. Put it in `front_offset` (default 0.16 m).

## 4. Check the `/odom` signs (2 minutes, robot on the floor, clear space toward the tag)
```bash
# terminal 1: driver (same as Phase 2)
ros2 launch robomaster_ros ep.launch conn_type:=sta video_resolution:=720 chassis_timeout:=0.6 tof_0:=true
# terminal 2: watch odometry speed
ros2 topic echo /odom --field twist.twist
# terminal 3: creep TOWARD the tag for ~2 s (drive_sign = -1, so negative x), then Ctrl+C
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05}}"
```
- `linear.x` negative while moving toward the tag → keep `odom_v_sign: -1.0`. Positive → set `+1.0`.
- `linear.x` and `angular.z` always 0 → set `odom_source: pose`.
- If the robot moved **away** from the tag, Phase 2's `drive_sign` no longer holds: set `drive_sign: 1.0` **and**
  re-check `odom_v_sign`.

## 5. Camera arm pose
The camera calibration only holds at one arm pose. Phase 2 saved `x 0.123, z 0.067` (set by hand, 2 cm above the
ToF). Set the arm the same way, check with `ros2 topic echo /arm_position`, and update `arm_x_saved`/`arm_z_saved`
if you choose a new pose. The node waits until the arm is at the saved pose, and brakes if it moves.

## 6. Range calibration
```bash
ros2 run rm_odom_ekf tag_detector --ros-args --params-file src/rm_odom_ekf/config/robot.yaml
# tag at 0.40, 0.70, 1.00, 1.30, 1.60 m from the chassis front (tape), one command each:
python3 scripts/calibrate_range.py --tape 0.40
python3 scripts/calibrate_range.py --fit       # -> copy range_scale and range_offset into robot.yaml
cd ~/ros2_ws && colcon build --symlink-install --packages-select rm_odom_ekf
```
Also set `camera_pitch_deg` if you moved the arm. Residuals in `--fit` should be within ±1 cm.

## 7. One dry run
Lift the wheels off the ground (box under the chassis) and run `ros2 launch rm_odom_ekf phase3.launch.py trial:=0`.
Check: WAITING → DRIVING, the terminal prints `d_est` decreasing, it ends with DONE. Delete `results/trials/trial_00_*`.

## 8. Trials 1 … 10
For N = 1..10:
1. Place the robot on the start marks, facing the tag. Arm at the saved pose.
2. Terminal 1: the driver (step 4). Terminal 2:
   ```bash
   ros2 launch rm_odom_ekf phase3.launch.py trial:=N
   ```
3. Wait for `Trial N done (EKF distance …): d_est = …`. **Don't touch the robot.**
4. Tape from the **chassis front midpoint** to the **tag face**, perpendicular, read to the millimetre:
   ```bash
   python3 scripts/add_measurement.py N 0.4xx
   ```
5. Note anything unusual (slip, tag lost, bump). A trial that ends in a safety stop is saved as `valid: false`.
   Re-run the same number; the analysis uses the newest valid run.

## 9. Results
```bash
python3 scripts/analyze_trials.py          # table + statistics + figures in results/
git add results && git commit -m "Phase 3 trials 1-10" && git push
```
Optional extra for Section 4 of the report: set `use_tag_range: false` (bearing only) in `robot.yaml`, rebuild, and
run one or two trials with `ros2 launch rm_odom_ekf phase3.launch.py trial:=1 results_dir:=~/SCE672-Phase3-RoboMaster-Odometry/results/bearing_only`
to show the along-track drift coming back. Set it back to `true` afterwards.

# Demo video checklist (Deliverable 2)

Video link for the report and slides: **[INSERT_YOUTUBE_VIDEO_URL]** (upload as *Unlisted* unless told otherwise).

## Before filming
- [ ] Battery > 50 %, steps 1–6 of [RUN_ON_ROBOT.md](RUN_ON_ROBOT.md) done, one dry run OK
- [ ] Phone on a tripod **side-on**, so the robot, the tag, the start line and the stop point are all in frame
- [ ] Tape measure lying on the floor along the path (the viewer can read distances)
- [ ] Screen recorder ready on Ubuntu (`Ctrl+Alt+Shift+R` in GNOME, or OBS)
- [ ] `rqt_image_view /tag/debug_image` open (green box around the tag = detection)

## Shots (about 60–90 s in total)
1. **Setup (5 s)**: the tag flat on the floor, the robot on the start line, a tape showing **1.50 m**.
2. **Terminal (5 s)**: `ros2 launch rm_odom_ekf phase3.launch.py trial:=N` being typed.
3. **Detection (5 s)**: debug image with the green box; terminal shows `WAITING -> DRIVING`.
4. **Motion (10–15 s)**: phone view: straight-line drive, slowing down, clean stop **before** the tag.
   Screen: `d_est` / `camera` / `odom-only` lines counting down.
5. **Stop (5 s)**: terminal `DRIVING -> HOLDING (EKF distance 0.40x m)`, then `Trial N done: d_est = …`.
6. **Ground truth (10 s)**: tape from the chassis front midpoint to the tag centre mark, close-up of the reading.
7. **Logging (5 s)**: `python3 scripts/add_measurement.py N 0.4xx` → printed error.
8. **Results (10 s, optional)**: Fig. 1 and Fig. 2 from `results/figures/`.

## Editing
- Phone view and screen recording **side by side** (as in Phase 2), synced on the moment the robot starts moving.
- Captions: "odometry prediction + camera correction (EKF)", "start 1.50 m", "stop 0.40 m", "trial N of 10".
- No music needed. Keep the real terminal text readable (1080p).

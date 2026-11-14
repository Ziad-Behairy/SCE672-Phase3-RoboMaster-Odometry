# GitHub setup (Deliverable 1)

Repository: **SCE672-Phase3-RoboMaster-Odometry** (public). Link for the report/slides: **[INSERT_GITHUB_REPO_URL]**

## Folder layout
```
src/rm_odom_ekf/      ROS 2 package (EKF, controller, detector, node, logger, tests)
theory/               equations, formula → line map, odometry role, spec PDF
scripts/              add_measurement, analyze_trials, calibrate_range, simulate_trials, formula_lines
results/              trials/ (one folder per trial), figures/, summary tables
docs/                 code explanation guide, robot runbook, video checklist, this file
```

## Windows: create the public repo and push (once)
Open a terminal in `C:\Users\Ziad Behairy\Projects\SCE672-Phase3-RoboMaster-Odometry` (Git Bash, PowerShell or
the VS Code terminal). The GitHub CLI is already signed in.
```bash
git init -b main
git add .
git commit -m "Phase 3: 5-state EKF, straight-line controller, trial logger and analysis"
gh repo create SCE672-Phase3-RoboMaster-Odometry --public --source . --push
gh repo view --web
```
Then replace `[INSERT_GITHUB_REPO_URL]` in `README.md` and the docs with the real link.

## Ubuntu: get the code, run the trials, push the results
```bash
cd ~ && git clone [INSERT_GITHUB_REPO_URL] SCE672-Phase3-RoboMaster-Odometry
cd ~/SCE672-Phase3-RoboMaster-Odometry
# ... calibration and trials (docs/RUN_ON_ROBOT.md) ...
git add src/rm_odom_ekf/config/robot.yaml results
git commit -m "Calibration and trials 1-10"
git push
```
Pushing from Ubuntu needs a sign-in: `gh auth login` (if `gh` is installed) or a personal access token as the password.

## Back on Windows
```bash
git pull
```

## Rules
- Commit `results/trials/*` exactly as the robot wrote them. Only `add_measurement.py` edits a summary.
- `results/_demo/` (simulated) is in `.gitignore` and never goes into the report.
- Commit after each session, so the history shows when each trial happened.

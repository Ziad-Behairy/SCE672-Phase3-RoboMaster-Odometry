# Formula map: each equation and where it is in the code (Deliverable 4)

Equations from [`theory_equations.md`](theory_equations.md) (Lectures 11A, 11B, 12A, 12B).
If the code changes and lines move, `python3 scripts/formula_lines.py` prints the new positions.

**ekf.py** = [`src/rm_odom_ekf/rm_odom_ekf/ekf.py`](../src/rm_odom_ekf/rm_odom_ekf/ekf.py) ·
**node** = [`src/rm_odom_ekf/rm_odom_ekf/phase3_node.py`](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py)

| # | Formula | Code | What it does | Without it |
|---|---|---|---|---|
| 1 | State $x=[p_x,p_y,\theta,v,\omega]^T$ (Lec 11A) | [ekf.py L18–19](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L18-L19) `IDX_X … IDX_TURN_RATE` | The five things we track. Speed and turn rate are states because the chassis lags behind `cmd_vel`. | `/odom` (which measures $v,\omega$) would have nothing in the state to compare against. |
| 2 | Motion model $f$ (Lec 11A) | [ekf.py L29–39](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L29-L39) `predict_state` | Prediction: drive at $v$ along $\theta$ for $dt$ and see where you end up. | Nothing fills the gaps between camera frames, or covers the time the tag is lost. |
| 3 | $F=\partial f/\partial x$ (Lec 11A sl. 15) | [ekf.py L42–56](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L42-L56) `motion_jacobian` | How errors spread: speed error → position error ($\cos\theta\,dt$), heading error → sideways error ($v\cos\theta\,dt$). | $P$ never links position with speed, so odometry can't correct position (row 6). |
| 4 | $G=\partial f/\partial u$ (Lec 11A) | [ekf.py L59–64](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L59-L64) `input_jacobian` | Unknown accelerations become uncertainty on $v,\omega$ through $GQG^T$. | The filter trusts its own speed forever and ignores `/odom`. |
| 5 | $P^-=FPF^T+GQG^T(+Q_{slip}dt)$ | [ekf.py L129–138](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L129-L138) `predict`; called at [node L198–204](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py#L198-L204) | Uncertainty grows while we only predict. | $P$ collapses, and the filter stops listening to the camera. |
| 6 | $h_{odom}=[v,\omega]$, $H_{odom}$ (Lec 11B) | [ekf.py L69–78](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L69-L78); [`correct_with_odom` L159–161](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L159-L161); called at [node L206–212](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py#L206-L212) | Odometry correction. $H$ just picks the speed and turn-rate rows. Position is corrected indirectly through the $p_{xv}$ term of $P$. | Speed is a pure guess from the command. Lag and stalls go unnoticed. |
| 7 | $d=\sqrt{\Delta x^2+\Delta y^2}$, $\varphi=\text{atan2}(\Delta y,\Delta x)-\theta$ (Lec 12A) | [ekf.py L81–87](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L81-L87) `expected_tag_reading` | What the camera should see if the estimate were right. | Nothing to compare the camera reading with. |
| 8 | $H_{tag}$ (Lec 12A) | [ekf.py L90–103](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L90-L103) `tag_jacobian` | The range row fixes along-track error; the bearing row fixes sideways error and heading. | The gain is the wrong shape, and corrections go into the wrong states. |
| 9 | Innovation $e=z-h(x^-)$ | [ekf.py L142](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L142) | How far the sensor disagrees with the prediction. | — |
| 10 | $\text{wrap}(\alpha)=\text{atan2}(\sin\alpha,\cos\alpha)$ (Lec 12B) | defined [L22–24](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L22-L24); bearing innovation [L143–144](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L143-L144); heading after update [L154](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L154); heading after predict [L137](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L137); predicted bearing [L86](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L86) | +179° and −179° are 2° apart, not 358°. | Crossing ±π gives a ~2π correction and the heading flips. **Not** applied to range, $P$ or $K$ (`test_range_error_is_never_wrapped`). |
| 11 | $S=HP^-H^T+R$ | [ekf.py L146](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L146) | Expected size of the disagreement: our doubt plus the sensor noise. | No way to weigh the sensor against the prediction. |
| 12 | Gate $e^TS^{-1}e\le 9.21$ | [ekf.py L148–150](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L148-L150) | Drops camera readings that can't be right (blur, wrong tag). | One bad detection moves the estimate by tens of centimetres. |
| 13 | $K=P^-H^TS^{-1}$ (scalar form $p_{i\varphi}/(p_{\varphi\varphi}+r_{\varphi\varphi})$) | [ekf.py L152](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L152) | How much to believe the sensor, state by state. Logged as `K00…K41` in `steps.csv`. | — |
| 14 | $x^+=x^-+Ke$ | [ekf.py L153](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L153) | Move the estimate toward the sensor by that amount. | — |
| 15 | $P^+=(I-KH)P^-$ | [ekf.py L155–156](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L155-L156) | We know more now, so $P$ shrinks. Symmetrised against rounding. | $P$ only grows, and the estimate chases sensor noise. |
| 16 | $d_{front}=\lVert m-p\rVert-\ell_{front}$, $\sigma_d=\sqrt{JPJ^T}$ | [ekf.py L174–186](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L174-L186) `distance_from_front` | The reported distance, from the same point the tape uses (chassis front), plus its σ. | Estimate and tape would refer to different points on the robot. |
| 17 | $e_{final}=d_{est}-d_{meas}$ | [scripts/add_measurement.py L42](../scripts/add_measurement.py#L42) | Error for each trial. | — |
| 18 | MAE, range, bias, σ², σ, cov | [scripts/analyze_trials.py](../scripts/analyze_trials.py) `stats_of`, `compute` | Statistics over all trials. | — |

## Checking the Jacobians (Lec 11A slide 18)
[`test_ekf.py`](../src/rm_odom_ekf/test/test_ekf.py) compares $F$, $G$ and $H_{tag}$ with central differences,
$(f(x+he_j)-f(x-he_j))/2h$ with $h=10^{-6}$:
```bash
cd src/rm_odom_ekf && python3 -m pytest -q test
```

## For Section 4 of the report: why the camera has to give range (Lec 12A slide 26)
Facing the tag, $\Delta y\approx0$, so the bearing row of $H_{tag}$ is $[\,0,\ -1/d,\ -1,\ 0,\ 0\,]$. Its $p_x$ entry is
zero, so bearing tells the filter nothing about how far along the line the robot is, and that is exactly the
direction wheel odometry drifts in. Only the range row $[-1,0,0,0,0]$ sees along-track error.
`test_bearing_alone_cannot_fix_along_track_error` shows this numerically. Setting `use_tag_range: false` shows it on
the robot.

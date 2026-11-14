# Formula map: every equation → the exact lines of code (Deliverable 4)

Equations from [`theory_equations.md`](theory_equations.md) (Lectures 11A, 11B, 12A, 12B). Each one is tagged in
the code with `[EQ <section>]`. If you edit the code and the line numbers move, run
`python3 scripts/formula_lines.py` to print the new positions.

Short names: **ekf.py** = [`src/rm_odom_ekf/rm_odom_ekf/ekf.py`](../src/rm_odom_ekf/rm_odom_ekf/ekf.py),
**node** = [`src/rm_odom_ekf/rm_odom_ekf/phase3_node.py`](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py).

| # | Formula | Where | What it does | What breaks without it |
|---|---|---|---|---|
| 1 | State $x=[p_x,p_y,\theta,v,\omega]^T$ (Lec 11A) | [ekf.py L15](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L15) (`PX, PY, TH, V, W`) | Names the 5 numbers the robot keeps track of. $v,\omega$ are states because the chassis takes time to reach the commanded speed. | With only the pose as states, `/odom` (which measures $v,\omega$) could not be used as a measurement. |
| 2 | Motion model $f$: $p_x{+}v\cos\theta\,dt$, $p_y{+}v\sin\theta\,dt$, $\theta{+}\omega dt$, $v{+}a\,dt$, $\omega{+}\alpha dt$ (Lec 11A) | [ekf.py L26–36](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L26-L36) | **Prediction**: "if I drive at speed $v$ for $dt$ seconds in direction $\theta$, here is where I'll be." | No guess between camera frames; the estimate jumps around and is lost when the tag drops out. |
| 3 | State Jacobian $F=\partial f/\partial x$ (Lec 11A slide 15) | [ekf.py L39–48](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L39-L48) | Tells $P$ how errors spread: a speed error becomes a position error ($\cos\theta\,dt$ entry); a heading error becomes a sideways error ($v\cos\theta\,dt$ entry). | $P$ would never link $p_x$ with $v$, so odometry could not correct position (row 6) and the uncertainty would be wrong. |
| 4 | Input Jacobian $G=\partial f/\partial u$ (Lec 11A) | [ekf.py L51–59](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L51-L59) | Turns the unknown accelerations $(a,\alpha)$ into uncertainty on $v,\omega$: $G Q_u G^T$. | No process noise on $v,\omega$: the filter would trust its own speed forever and ignore `/odom`. |
| 5 | Covariance prediction $P^-=FPF^T+GQ_uG^T(+Q_{pose}dt)$ | [ekf.py L118–124](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L118-L124); called at [node L181–186](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py#L181-L186) | Uncertainty grows while the robot only predicts (wheel slip, lag). | $P$ shrinks to zero; later camera readings get ignored (overconfident filter). |
| 6 | $h_{odom}(x)=[v,\omega]$, $H_{odom}=\begin{bmatrix}0&0&0&1&0\\0&0&0&0&1\end{bmatrix}$ (Lec 11B) | [ekf.py L64–73](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L64-L73); update at [ekf.py L143–145](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L143-L145); called at [node L188–194](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py#L188-L194) | **Odometry correction**: `/odom` measures speed and turn rate, so $H$ picks those two states. Position is corrected *indirectly* through $p_{xv}$ in $P$. | Speed is only a guess from the command; inner-loop lag and stalls go unnoticed. |
| 7 | Range–bearing model $d=\sqrt{\Delta x^2+\Delta y^2}$, $\varphi=\text{atan2}(\Delta y,\Delta x)-\theta$ (Lec 12A) | [ekf.py L76–82](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L76-L82) | What the camera *should* see if our estimate were right. | Nothing to compare the camera with: no correction. |
| 8 | $H_{tag}=\begin{bmatrix}-\Delta x/d&-\Delta y/d&0&0&0\\ \Delta y/q&-\Delta x/q&-1&0&0\end{bmatrix}$ (Lec 12A) | [ekf.py L85–94](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L85-L94) | How a small change in pose changes $d$ and $\varphi$. Row 1 (range) fixes **along-track** drift; row 2 (bearing) fixes **sideways** error and heading. | Kalman gain would be wrong-shaped; corrections go to the wrong states. |
| 9 | Innovation $e=z-h(x^-)$ | [ekf.py L128](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L128) | The disagreement between the sensor and the prediction. | — (this *is* the correction signal) |
| 10 | **Angle wrap** $\text{wrap}(\alpha)=\text{atan2}(\sin\alpha,\cos\alpha)$ (Lec 12B) | defined [ekf.py L19–21](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L19-L21); bearing innovation [L129–130](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L129-L130); $\theta^+$ after update [L138](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L138); $\theta$ after predict [L123](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L123); predicted bearing [L81](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L81) | Keeps angles in $(-\pi,\pi]$. +179° vs −179° is a 2° difference, not 358°. | A ±π crossing gives a ~2π innovation: the robot "corrects" itself into a spin. **Never** applied to the distance innovation, $P$ or $K$ (test: `test_range_innovation_is_not_wrapped`). |
| 11 | Innovation covariance $S=HP^-H^T+R$ | [ekf.py L131](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L131) | Expected size of the disagreement: our own doubt + sensor noise. | No way to weigh the sensor against the prediction. |
| 12 | Gate $e^TS^{-1}e \le 9.21$ (χ², 99 %) | [ekf.py L133–135](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L133-L135) | Throws away a camera reading that is impossibly far from the prediction (wrong tag, blur). | One bad detection yanks the estimate by tens of cm. |
| 13 | Kalman gain $K=P^-H^TS^{-1}$ (scalar form $K_i=p_{i\varphi}/(p_{\varphi\varphi}+r_{\varphi\varphi})$, Lec 12A) | [ekf.py L136](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L136) | How much to believe the sensor, per state: big when we are unsure, small when the sensor is noisy. | — (logged every step: columns `K00..K41` of `steps.csv`, `K_last_tag` in `summary.json`) |
| 14 | State update $x^+=x^-+Ke$ | [ekf.py L137](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L137) | Moves the estimate toward the sensor by the right amount. | — |
| 15 | Covariance update $P^+=(I-KH)P^-$ | [ekf.py L139–140](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L139-L140) | We are now more certain; symmetrised against rounding. | $P$ only grows; gains stay huge; the estimate chases sensor noise. |
| 16 | Final distance and its σ: $d_{front}=\lVert m-p\rVert-\ell_{front}$, $\sigma_d=\sqrt{JPJ^T}$ | [ekf.py L156–167](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L156-L167) | The graded number ($d_{est}$) measured from the **same reference point as the tape** (chassis front), plus how sure the EKF is. | Estimate and tape would refer to different points on the robot (spec, required work 2). |
| 17 | Error $e_{final}=d_{est}-d_{meas}$ | [scripts/add_measurement.py L42](../scripts/add_measurement.py#L42) | Per-trial error (spec, required work 3). | — |
| 18 | MAE, range, bias, σ², σ, cov | [scripts/analyze_trials.py `stats_of` / `compute`](../scripts/analyze_trials.py) | Deliverable 7 statistics. | — |

## The check from Lecture 11A slide 18

Every Jacobian is compared with central differences
$\partial f_i/\partial x_j \approx (f_i(x+he_j)-f_i(x-he_j))/2h$, $h=10^{-6}$, in
[`test_ekf.py`](../src/rm_odom_ekf/test/test_ekf.py) (`test_F_…`, `test_G_…`, `test_H_tag_…`). Run on Windows or Ubuntu:

```bash
cd src/rm_odom_ekf && python3 -m pytest -q test
```

## Section 4 of the report: why the camera must give range, not just bearing (Lec 12A slide 26)

Driving straight at the tag, $\Delta y\approx 0$, so the bearing row of $H_{tag}$ is $[\,0,\ -1/d,\ -1,\ 0,\ 0\,]$:
it has **zero** in the $p_x$ column. Bearing alone tells the robot nothing about how far along the line it is,
which is exactly the direction wheel odometry drifts. The range row $[-1,\ 0,\ 0,\ 0,\ 0]$ is the only thing that
sees along-track error. The test `test_bearing_only_barely_fixes_along_track` proves it numerically, and
`use_tag_range: false` in `robot.yaml` lets you show it on the robot as an extra trial.

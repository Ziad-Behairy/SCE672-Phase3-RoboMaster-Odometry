# The role of odometry in this project (Deliverable 5)

## In one sentence
`/odom` tells the robot **how fast it is going and turning** ($v$, $\omega$) ten times a second; the EKF turns
that into a **prediction** of where it is, and the camera **corrects** that prediction.

## Where `/odom` enters the code

| Step | Code | What happens |
|---|---|---|
| Read the message | [`phase3_node.py` `on_odom`, L136–153](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py#L136-L153) | Takes `twist.twist.linear.x` and `twist.twist.angular.z` from `nav_msgs/Odometry` (~10 Hz). `odom_v_sign = -1` because the driver's +x points away from the camera on this robot (Phase 2 wheels-in-the-air test). If the driver leaves `twist` empty, `odom_source: pose` differentiates the pose instead. |
| Correct $v,\omega$ | [`ekf.py` `update_odom`, L143–145](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L143-L145), $H_{odom}$ at [L64–68](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L64-L68) | $z=[v,\omega]$, $H_{odom}$ picks states 4 and 5. Noise $R_{odom}=\text{diag}(0.02^2, 0.03^2)$ from `robot.yaml`. |
| Predict the pose | [`ekf.py` `predict`, L118–124](../src/rm_odom_ekf/rm_odom_ekf/ekf.py#L118-L124) | The corrected $v,\omega$ move $p_x,p_y,\theta$ forward: $p_x \mathrel{+}= v\cos\theta\,dt$, … |
| Odometry-only baseline | [`on_odom` L152](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py#L152) → `d_odom_only` | Adds up $v\,dt$ with **no** camera, so the report can show how far pure odometry drifts (blue EKF vs green odometry-only in Figs. 2, 3 and 5). |

## Why odometry alone is not enough

1. **It is blind to position.** Encoders (and the IMU inside `/odom`) measure motion, never *where* the robot is.
   Position is a sum of $v\,dt$ steps, so every small speed error is added up and never removed. Error grows with
   distance travelled.
2. **Wheel slip.** Mecanum rollers slip, especially when starting and braking. The wheels turn more than the robot
   moves, so odometry over-reads along-track distance. It thinks it is closer to the tag than it is.
3. **Lag.** The chassis speed loop takes ~0.3 s to reach a command. The model covers this with $v$ as a state and
   $u=(v_{cmd}-v)/\tau$ ([node L181–186](../src/rm_odom_ekf/rm_odom_ekf/phase3_node.py#L181-L186)).
4. **Unknown start.** Odometry starts at "zero", wherever the robot was put down. Only the camera sees the tag.

So the error that hurts us most, **along-track distance**, is exactly the one odometry cannot see. The camera's
**range** measurement is the only thing that can (see the end of [FORMULA_MAP.md](FORMULA_MAP.md)).

## Why odometry is still needed

- The camera runs at ~12–15 Hz, drops frames, blurs when moving, and can lose the tag up close. Between and
  during those gaps, the odometry **prediction** keeps the estimate smooth and moving.
- Odometry noise is small over short time. The EKF uses it for the short term and the camera for the long term.
- Through $P$ the two help each other: $F$ links speed and position ($p_{xv}$), so a speed correction from
  `/odom` also nudges $p_x$.

## Why we do NOT also fuse `/imu` (Lecture 11B, slide 24)

The `robomaster_ros` driver **already fuses the IMU and the wheel encoders** to make `/odom`. If we also fed
`/imu` into the EKF, the same gyro reading would be counted twice. The filter would think it had two independent
witnesses when it really had one. $P$ would shrink too fast, the filter would become overconfident, and it would
start ignoring the camera.

The node never subscribes to `/imu` (check: `grep -rn imu src/` finds only comments). The correct alternative,
fusing raw `/joint_states_p` + `/imu` *instead of* `/odom`, is noted as future work.

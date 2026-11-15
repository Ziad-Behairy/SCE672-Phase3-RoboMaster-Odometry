| Variant | MAE (cm) | bias (cm) | σ (cm) | range (cm) | worst (cm) |
|---|---:|---:|---:|---:|---:|
| EKF as run (predict + odom + range + bearing) | 0.28 | -0.10 | 0.32 | 0.96 | 0.48 |
| EKF, camera bearing only | 1.98 | -0.95 | 2.63 | 9.80 | 4.92 |
| EKF, no camera while driving | 2.53 | -1.52 | 3.12 | 11.28 | 6.20 |
| EKF, camera lost below 0.90 m | 1.86 | +1.17 | 1.64 | 4.64 | 2.72 |
| camera only (last reading, no filter) | 0.30 | -0.15 | 0.33 | 1.07 | 0.55 |
| EKF, camera noise R x3 | 0.25 | +0.16 | 0.31 | 1.14 | 0.68 |
| EKF, camera noise R / 3 | 0.30 | -0.13 | 0.34 | 1.06 | 0.54 |

# iAHRS IMU Mount Position

Physical position of the IMU on this robot, relative to `base_link`.

| Axis | Value (m) | Notes |
|------|-----------|-------|
| x    | 0.00      | on robot center line |
| y    | 0.00      | on robot center line |
| z    | 0.58      | height above base_link |

The TF frame is `base_link` -> `imu_link`.

## How each launch applies this position

There are two launch files with different TF strategies:

1. **[iahrs_driver_launch.xml](../iahrs_driver/launch/iahrs_driver_launch.xml)** —
   `m_bSingle_TF_option=true`. The driver publishes a dynamic `base_link -> imu_link`
   TF whose translation is `(0, 0, tf_translation_z)` and whose rotation is the live
   IMU orientation. `tf_translation_z` is set to **0.58** so the TF carries the real
   mount height.
   - In the source ([iahrs_driver.cpp](../iahrs_driver/src/iahrs_driver.cpp),
     `single_tf_option` branch) `x` and `y` are hardcoded to `0.0`; only `z` is a
     parameter. A non-zero `x`/`y` offset would require a source change.

2. **[iahrs_driver.py](../iahrs_driver/launch/iahrs_driver.py)** — publishes a static
   `base_link -> imu_link` TF with translation **(0, 0, 0.58)**, matching the real mount.
   The driver's dynamic TF is disabled here (`m_bSingle_TF_option=False`) so a fixed
   static mount TF is used for 3D SLAM IMU integration.

## Note: 2D SLAM (Cartographer) is NOT used in this project

This project does not use 2D SLAM (Cartographer). The original FITO code pinned the
`iahrs_driver.py` static TF to translation `(0, 0, 0)` to satisfy Cartographer's
colocation constraint (IMU frame colocated with the tracking frame); that constraint
does not apply here, so the static TF has been corrected to the real mount
`(0, 0, 0.58)`. Both launch files now carry the physical mount height.

## Changing the mount height

- Edit `tf_translation_z` in `iahrs_driver_launch.xml`.
- A non-zero `x`/`y` offset requires a source change (currently hardcoded to 0).

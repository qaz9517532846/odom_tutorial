# odom_tutorial
How to do transform odom data from each localization Algorithm. 

Suscriber laser_matcher position transform to odom data.
``` bash
$ rosrun odom_tutorial agv_odom
```

Suscriber laser_matcher position and encoder transform to odom data.
``` bash
$ rosrun odom_tutorial odom_amcl
```

Suscriber orb_slam position transform to odom data.
``` bash
$ rosrun odom_tutorial odom_pose
```

Suscriber orb_slam position transform to odom data(base_link).
``` bash
$ rosrun odom_tutorial odom_pose_nav
```

Suscriber laser_matcher and encoder position transform to odom data.
``` bash
$ rosrun odom_tutorial odom_vel
```

Encoder data publisher test.

``` bash
$ rosrun odom_tutorial vel_pub
```

Encoder data to odom data.

``` bash
$ rosrun odom_tutorial encoder_odom
```

Encoder and IMU data to odom data.

``` bash
$ rosrun odom_tutorial encoder_imu_odom
```

Encoder and IMU , AMCL pose to odom data.

``` bash
$ rosrun odom_tutorial encoder_imu_AMCL_odom
```

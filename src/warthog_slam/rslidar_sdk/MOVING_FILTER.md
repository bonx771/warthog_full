# Filter moving objects while Warthog drives

`start_dynamic_filter_moving.launch` uses Warthog odometry. The base controller
publishes `/warthog_velocity_controller/odom` with pose `odom -> base_link`.
The launch starts the LiDAR driver and publishes a static `base_link -> rslidar`
transform unless configured otherwise. It does not modify Warthog files.

In a separate terminal after the Warthog base controller is running:

```bash
source /home/cuong/rslidar_ws/devel/setup.bash
roslaunch rslidar_sdk start_dynamic_filter_moving.launch
```

RViz opens with only the filtered moving cloud enabled; pass `rviz:=false` if
another RViz window is already open. If another launch already starts the
LiDAR or publishes `base_link -> rslidar`, use `run_lidar:=false` or
`publish_lidar_tf:=false` respectively. Warthog's outdoor obstacle launch
provides both this TF and `/outdoor_waypoint_nav/odometry/filtered`; when
running alongside it, use:

```bash
roslaunch rslidar_sdk start_dynamic_filter_moving.launch \
  run_lidar:=false publish_lidar_tf:=false \
  odom_topic:=/outdoor_waypoint_nav/odometry/filtered
```

When running standalone, set the mount arguments (`lidar_x`, `lidar_y`,
`lidar_z`, `lidar_roll`, `lidar_pitch`, `lidar_yaw`) to the measured LiDAR
installation. The `0.93 m` default is taken from Warthog's
`outdoor_waypoint_obstacle.launch` and may not fit another mount.

The filter publishes `/rslidar_points_static_moving` and
`/rslidar_points_dynamic_moving`. Both retain the original LiDAR frame and
point fields. The latter contains unstable points, including newly seen static
surfaces. To create a separate 2-D scan for mapping:

```bash
roslaunch rslidar_sdk helios32_static_to_scan.launch \
  run_dynamic_filter:=false run_lidar:=false rviz:=false \
  cloud_topic:=/rslidar_points_static_moving scan_topic:=/scan_static_moving
```

Keep the raw cloud or raw scan as the input to collision avoidance. A person
removed from the static mapping cloud is still a physical obstacle.

## Method and tuning

The node interpolates Warthog odometry at the cloud timestamp and at the
`timestamp` of each Helios 32 point. It transforms points into `odom`, then
checks how long each 3-D voxel remains occupied across successive scans. The
default `0.12 m` voxel must be observed in at least 5 scans over at least
`2.0 s`. Once confirmed, a voxel stays eligible through brief missed returns
(`evidence_timeout_s`, default 5 s). A face-adjacent voxel can also support a
wall point if it was confirmed within `neighbor_support_gap_s` (default 1 s).
This reduces holes when the robot moves at 0.2–0.4 m/s and the viewing angle
shifts. When the evidence map fills, the oldest cells are evicted instead of
clearing the entire map. If odometry or the LiDAR mounting TF is missing, the
node drops the cloud and warns instead of using an uncorrected pose.

Smaller `voxel_size_m` and larger `min_static_age_s` reject slower motion, but
new static surfaces take longer to appear and odometry errors reject more
walls. Compare the two output topics in RViz and tune with recorded real data.
Neighbor support can retain a pedestrian point immediately next to an already
confirmed wall. A newly encountered wall still needs time to be confirmed;
these settings cannot guarantee a hole-free map at every speed.
Very slow or stationary people can still become part of the background. For
reliable removal regardless of speed, compare against a surveyed static map
or add tracked object detection; odometry alone cannot tell whether an object
that stays in one place is a person or a wall.

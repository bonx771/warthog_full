# Filter moving points while the robot is stationary

Keep both the robot and the LiDAR stationary. This filter compares the range
returned by each angular cell across scans; it does not compensate for robot
motion. The SDK publishes XYZIRT points, and the filter uses `ring` and preserves
all fields, including `timestamp`, in both output clouds.

## Run

Start the LiDAR driver, filter and filtered-cloud RViz view together:

```bash
source /home/tung/rslidar_ws/devel/setup.bash
roslaunch rslidar_sdk start_dynamic_filter.launch
```

If the LiDAR driver is already running, start only the filter and its RViz view:

```bash
source /home/tung/rslidar_ws/devel/setup.bash
roslaunch rslidar_sdk start_dynamic_filter.launch run_lidar:=false
```

Use `rviz:=false` to run without opening RViz. Start with a clear scene, allow
the background to stabilize, then have a person walk through the scan area.

## View the output

RViz uses fixed frame `rslidar` and opens with only the filtered cloud enabled.
No TF or axes display is required.

| RViz display | Topic | Meaning |
| --- | --- | --- |
| RS LiDAR (filtered) | `/rslidar_points_static` | Points accepted by the temporal background filter |
| RS LiDAR (raw) | `/rslidar_points` | Original cloud; enable to compare |
| Removed points | `/rslidar_points_dynamic_candidates` | Rejected points; enable to see them in red |

Removed points include new or unstable observations and are not guaranteed to
belong to moving objects. The filter removes observed points; it does not fill
in the background hidden behind a person.

## Adjust the filter

- `stable_frames:=3`: accept a new background cell after three matching
  observations.
- `foreground_persistence_frames:=30`: replace the learned background with a
  nearer surface after 30 matching observations. At 10 Hz, this is about 3 s.
  Increasing this value rejects a foreground obstruction for longer.
- `clearing_persistence_frames:=3`: accept a newly revealed, farther background
  after three matching observations.
- `range_tolerance_m:=0.20` and `range_tolerance_ratio:=0.005`: allow range
  changes within `0.20 m + 0.005 * min(current range, reference range)`.

For example, keep rejecting a person who pauses in front of a learned wall for
longer before accepting them as a new background:

```bash
roslaunch rslidar_sdk start_dynamic_filter.launch run_lidar:=false foreground_persistence_frames:=100
```

This is a temporal filter, not a person/vehicle classifier. Slow motion within
the range tolerance, objects present during startup, and objects that remain
long enough can be accepted as static. A real scan must be used to tune these
parameters for the scene.

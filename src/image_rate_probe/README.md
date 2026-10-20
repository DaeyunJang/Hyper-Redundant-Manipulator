# image_rate_probe

Low-overhead C++ subscribers for separating RealSense capture rate from ROS 2
large-message delivery rate. Run only one probe at a time and stop RViz, RQT,
the estimator, and other topic subscribers for a clean camera-only baseline.

Build and source:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select image_rate_probe
source install/setup.bash
```

Measure an Image topic:

```bash
ros2 run image_rate_probe image_rate_probe --ros-args \
  -p topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p expected_fps:=30.0 \
  -p report_interval_sec:=5.0 \
  -p qos_depth:=1
```

Change `topic` to `/camera/camera/depth/image_rect_raw` or
`/camera/camera/color/image_rect_raw` for an alignment-independent comparison.

Measure the lightweight depth metadata stream:

```bash
ros2 run image_rate_probe metadata_rate_probe --ros-args \
  -p topic:=/camera/camera/depth/metadata \
  -p expected_fps:=30.0 \
  -p report_interval_sec:=5.0
```

`arrival` is the callback rate measured with a monotonic wall clock. `header`
uses message timestamps. `estimated_missing` counts nominal 30 Hz periods that
were skipped between received timestamps. The Image probe never reads or copies
individual pixels; it only inspects message metadata and the existing data
buffer size.

Use `qos_depth:=10` only as an A/B test for reader-queue loss. A depth of 1 is
the desired latest-frame behavior for the estimator.

## Complete estimation pipeline

```bash
ros2 run image_rate_probe pipeline_rate_probe
```

Reports color/aligned depth/crop/skeleton/angle/MarkerArray rates and source
timestamp age p50/p95 every 5 s. It counts repeated/backwards stamps and estimates
missing 30 Hz periods. Run on the camera host with the same ROS clock. The first
window includes startup/discovery, so judge steady windows. It subscribes to
visualization outputs (thus activating their publication); use
`--ros-args -p visualization:=false` for camera/angle inputs only.

See [the real-time guide](../../docs/REALTIME_ESTIMATION.md) for the Fast DDS
image-sized SHM configuration and controlled test results.

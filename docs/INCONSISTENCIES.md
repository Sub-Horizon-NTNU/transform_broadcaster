# Known inconsistencies

Reviewed on 2026-09-18. These notes record differences found by reading the repository; no runtime or hardware validation was performed. Existing code and instructions are preserved.

## Autopilot input differs from the original README

The original README describes ArduPilot `/ap/pose/filtered` and an ENU-to-NED conversion. [DynamicFramePublisher.hpp](../include/broadcasters/DynamicFramePublisher.hpp) instead subscribes to PX4 `/fmu/out/vehicle_odometry`. Its position is already NED, and the code reorders the quaternion from PX4's `[w,x,y,z]` layout.

## Camera parameters are read but not applied to the transform

[The launch file](../launch/transform_broadcaster.launch.py) exposes `camera_offset_x/y/z`, and [TransformBroadcasterNode.hpp](../include/TransformBroadcasterNode.hpp) reads them. The actual `usv_ned -> camera` transform is still constructed with literal offsets `0.41, -0.06, 0.0`. Changing launch arguments therefore does not currently change that transform. Defaults in the launch file and node also differ.

These are observations from source, not a request to change calibration or coordinate conventions.

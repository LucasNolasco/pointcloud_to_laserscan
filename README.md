# ROS pointcloud <-> laserscan converters

Fork from [ros-perception/pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan). This fork adds a feature for correcting the pointcloud height based on IMU data, which allows undesired points to be discarded even if the robot is inclinated. This package provides components to convert `sensor_msgs/PointCloud2` messages to `sensor_msgs/LaserScan` messages and back.

## pointcloud\_to\_laserscan::PointCloudToLaserScanNode

This component projects `sensor_msgs/PointCloud2` messages into `sensor_msgs/LaserScan` messages.

### Published Topics

* `scan` (`sensor_msgs/LaserScan`) - The output laser scan.

### Subscribed Topics

* `cloud_in` (`sensor_msgs/PointCloud2`) - The input point cloud. No input will be processed if there isn't at least one subscriber to the `scan` topic.

* `imu/data` (`sensor_msgs/imu`) - Data provided by the IMU module. If no information is published on this topic, the correction on the pointcloud height won't be applied.

### Parameters

* `min_height` (double, default: 0.0) - The minimum height to sample in the point cloud in meters.
* `max_height` (double, default: 1.0) - The maximum height to sample in the point cloud in meters.
* `angle_min` (double, default: -π/2) - The minimum scan angle in radians.
* `angle_max` (double, default: π/2) - The maximum scan angle in radians.
* `angle_increment` (double, default: π/360) - Resolution of laser scan in radians per ray.
* `queue_size` (double, default: detected number of cores) - Input point cloud queue size.
* `scan_time` (double, default: 1.0/30.0) - The scan rate in seconds. Only used to populate the scan_time field of the output laser scan message.
* `range_min` (double, default: 0.45) - The minimum ranges to return in meters.
* `range_max` (double, default: 4.0) - The maximum ranges to return in meters.
* `target_frame` (str, default: none) - If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.
* `transform_tolerance` (double, default: 0.01) - Time tolerance for transform lookups. Only used if a `target_frame` is provided.
* `use_inf` (boolean, default: true) - If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.

## pointcloud\_to\_laserscan::LaserScanToPointCloudNode

This component re-publishes `sensor_msgs/LaserScan` messages as `sensor_msgs/PointCloud2` messages.

### Published Topics

* `cloud` (`sensor_msgs/PointCloud2`) - The output point cloud.

### Subscribed Topics

* `scan_in` (`sensor_msgs/LaserScan`) - The input laser scan. No input will be processed if there isn't at least one subscriber to the `cloud` topic.

### Parameters

* `queue_size` (double, default: detected number of cores) - Input laser scan queue size.
* `target_frame` (str, default: none) - If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.
* `transform_tolerance` (double, default: 0.01) - Time tolerance for transform lookups. Only used if a `target_frame` is provided.
* `use_inf` (boolean, default: true) - If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.

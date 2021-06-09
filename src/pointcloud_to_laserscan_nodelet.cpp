/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <limits>
#include <pluginlib/class_list_macros.h>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PointStamped.h>

namespace pointcloud_to_laserscan
{
PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet()
{
}

void PointCloudToLaserScanNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
  private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  private_nh_.param<double>("angle_min", angle_min_, -M_PI);
  private_nh_.param<double>("angle_max", angle_max_, M_PI);
  private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  private_nh_.param<double>("range_min", range_min_, 0.0);
  private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);
  private_nh_.param<bool>("use_inf", use_inf_, true);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
  }

  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                               boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));

  imu_sub = nh_.subscribe("/imu/data", 10, &PointCloudToLaserScanNodelet::imuCallback, this);
  tf2_.reset(new tf2_ros::Buffer());
  tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
  // pose_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::Imu>(imu_sub, *tf2_, "base_link", input_queue_size_, nh_));
  // pose_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::imuCallback, this, _1));

  pitch_ = 0;
  // imu_mutex.lock();

  valid_imu.store(false);
}

void PointCloudToLaserScanNodelet::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // tf2::Quaternion quat;
  // tf2::convert(msg.get()->orientation, quat);
  // tf2Scalar temp_roll, temp_pitch, temp_yaw;
  // tf2::Matrix3x3(quat).getRPY(temp_roll, temp_pitch, temp_yaw);

  // ROS_INFO("Pitch: %g", pitch_);
  
  // pitch_ = - (M_PI_2 + atan2(msg.get()->linear_acceleration.z, msg.get()->linear_acceleration.x));
  // ROS_INFO("Pitch: %g, z: %g, x: %g", pitch_, msg.get()->linear_acceleration.z, msg.get()->linear_acceleration.x);

  // geometry_msgs::Quaternion orientation;
  // StatelessOrientation::computeOrientation(msg.get()->linear_acceleration, orientation);
  
  // tf2Scalar temp_roll, temp_pitch, temp_yaw;
  // tf2::Quaternion quat;
  // tf2::fromMsg(orientation, quat);
  // tf2::Matrix3x3(quat).getRPY(temp_roll, temp_pitch, temp_yaw);

  // if (temp_roll > 0) {
  //   pitch_ = - (temp_roll - M_PI);
  // }
  // else if (temp_roll < 0) {
  //   pitch_ = - (temp_roll + M_PI);
  // }

  // ROS_INFO("Magic pitch: %g, Roll: %g, Pitch: %g, Yaw: %g", pitch_, temp_roll, temp_pitch, temp_yaw);

  double lidar_z, lidar_x;
  try {
    geometry_msgs::TransformStamped t_lidar = tf2_->lookupTransform("base_link", "rslidar", ros::Time(0));
    lidar_z = t_lidar.transform.translation.z;
    lidar_x = t_lidar.transform.translation.x;
  } catch (tf2::TransformException &ex) {
    lidar_z = 0;
    lidar_x = 0;
    ROS_INFO("Couldn't transform rslidar to base_link: %s", ex.what());
  }

  tf2::Quaternion original_rotation;
  tf2::convert(msg.get()->orientation, original_rotation);
  tf2Scalar orig_roll, orig_pitch, orig_yaw;
  tf2::Matrix3x3(original_rotation).getRPY(orig_roll, orig_pitch, orig_yaw);

  ROS_INFO("Original - Roll: %g, Pitch: %g, Yaw: %g", orig_roll, orig_pitch, orig_yaw);

  tf2Scalar temp_roll = 0, temp_pitch = 0, temp_yaw = 0;
  try {
    sensor_msgs::Imu imu_in = *msg.get();
    geometry_msgs::TransformStamped t_in = tf2_->lookupTransform("base_link", msg.get()->header.frame_id, imu_in.header.stamp);

    Eigen::Quaternion<double> r(t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    Eigen::Transform<double,3,Eigen::Affine> t(r);

    Eigen::Vector3d rotated_orientation = t * Eigen::Vector3d(
        orig_roll, orig_pitch, orig_yaw);

    temp_roll = rotated_orientation.x();
    temp_pitch = rotated_orientation.y();
    temp_yaw = rotated_orientation.z();

    // tf2::Quaternion quat;
    // tf2::convert(orientation_base_link, quat);
    // tf2::Matrix3x3(quat).getRPY(temp_roll, temp_pitch, temp_yaw);

    // sensor_msgs::Imu imu_in = *msg.get();
    // geometry_msgs::TransformStamped t_in = tf2_->lookupTransform("base_link", msg.get()->header.frame_id, imu_in.header.stamp);
  
    // Eigen::Quaternion<double> r(t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    // Eigen::Transform<double,3,Eigen::Affine> t(r);
    
    // Eigen::Vector3d rotated_rpy = t * Eigen::Vector3d(
    //   orig_roll, orig_pitch, orig_yaw);

    // temp_roll = rotated_rpy.x();
    // temp_pitch = rotated_rpy.y();
    // temp_yaw = rotated_rpy.z();
    
    // tf2::Quaternion quat(orientation.x(), orientation.y(), orientation.z(), orientation.w());
    // tf2::convert(imu_out.orientation, quat);
    // tf2::Matrix3x3(quat).getRPY(temp_roll, temp_pitch, temp_yaw);
  }
  catch (tf2::TransformException& ex) {
    ROS_INFO("Failure %s\n", ex.what()); //Print exception which was caught

    pitch_ = 0;
  }

  float bias;
  if (imu_bias.size() > 0) {
    bias = bias_sum / imu_bias.size();
    if (imu_bias.size() < 100) {
      bias_sum = bias_sum - imu_bias[0] + pitch_;
    }
  }
  else {
    bias = 0;
    bias_sum = pitch_;
  }

  if (imu_bias.size() < 100) {
    imu_bias.push_back(pitch_);
  }

  if (temp_pitch > 0) {
    pitch_ = (temp_pitch - M_PI) + 0.1;
  }
  else if (temp_pitch < 0) {
    pitch_ = (temp_pitch + M_PI) + 0.1;
  }

  // float pitch_copy;
  // if (pitch_ > 0) {
  //   pitch_copy = static_cast<float>(sqrt(pitch_));
  // }
  // else {
  //   pitch_copy = static_cast<float>(-sqrt(-1 * pitch_));
  // }

  lidar_height = lidar_z - (cos(pitch_) * lidar_z - sin(pitch_) * lidar_x);
  ROS_INFO("Magic pitch: %g, Roll: %g, Pitch: %g, Yaw: %g", pitch_, temp_roll, temp_pitch, temp_yaw);
  ROS_INFO_STREAM("Timestamp: " << msg.get()->header.stamp);
  ROS_INFO_STREAM("Height: " << lidar_height);

  valid_imu.store(true);

  imu_stamp = msg.get()->header.stamp;

  // imu_mutex.unlock();
}

void PointCloudToLaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
    sub_.subscribe(nh_, "cloud_in", input_queue_size_);
  }
}

void PointCloudToLaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
    sub_.unsubscribe();
  }
}

void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
                                                                             << message_filter_->getTargetFramesString()
                                                                             << " at time " << cloud_msg->header.stamp
                                                                             << ", reason: " << reason);
}

void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // build laserscan output
  sensor_msgs::LaserScan output;
  output.header = cloud_msg->header;
  if (!target_frame_.empty())
  {
    output.header.frame_id = target_frame_;
  }

  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;

  // Transform cloud if necessary
  if (!(output.header.frame_id == cloud_msg->header.frame_id))
  {
    try
    {
      cloud.reset(new sensor_msgs::PointCloud2);
      tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
      cloud_out = cloud;
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
    cloud_out = cloud_msg;
  }

  // ros::Duration wait_time(0.05);
  // while(!imu_mutex.try_lock()) {
  //   wait_time.sleep();
  // }

  // Iterate through pointcloud
  // if (fabs(imu_stamp.toSec() - cloud_msg->header.stamp.toSec()) < 0.05) {
  ROS_INFO_STREAM("======= NEW CLOUD, timestamp: " << cloud_msg->header.stamp << " =======");
  // float pitch_copy;
  // if (pitch_ > 0) {
  //   pitch_copy = static_cast<float>(sqrt(pitch_));
  // }
  // else {
  //   pitch_copy = static_cast<float>(-sqrt(-1 * pitch_));
  // }

  float pitch_copy = static_cast<float>(pitch_);

  float pitch_cos = cos(pitch_copy);
  float pitch_sin = sin(pitch_copy);
  // if (sqrt(fabs(pitch_)) < 0.1) {
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    valid_imu.store(false);
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_)
    {
      NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

    // float pitch_copy = static_cast<float>(pitch_);
    // float pitch_cos = cos(pitch_copy);
    // float pitch_sin = sin(pitch_copy);
    float rotated_z = pitch_cos * (*iter_z) - pitch_sin * (*iter_x) - lidar_height;
    if (rotated_z > max_height_ || rotated_z < min_height_) {
      // ROS_INFO("Measured Z: %f, Measured X: %f Rotated Z: %f, Rotation: %f", *iter_z, *iter_x, rotated_z, pitch_copy);
      continue;
    }

    // if (fabs(pitch_copy) > 0.08) {
    //   continue;
    // }

    // float rotated_x = pitch_sin * (*iter_z) + pitch_cos * (*iter_x);
    // double rotated_range = hypot(rotated_x, *iter_y);
    // if (rotated_range < range_min_ || rotated_range > range_max_) {
    //   ROS_INFO("Rotated range exceeded");
    //   continue;
    // }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }
  // }
  pub_.publish(output);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)

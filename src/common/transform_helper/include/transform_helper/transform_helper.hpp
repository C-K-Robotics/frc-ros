#ifndef TRANSFORM_HELPER__TRANSFORM_HELPER_HPP_
#define TRANSFORM_HELPER__TRANSFORM_HELPER_HPP_

#include <memory>
#include <string>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace CKR
{
class TransformHelper
{
public:
  explicit TransformHelper(rclcpp::Node & node);

  bool lookup_transform(
    const std::string & source_frame, const std::string & target_frame,
    geometry_msgs::msg::TransformStamped & transform, const rclcpp::Time & time);

  void send_transform(const geometry_msgs::msg::TransformStamped & msg);

  template<typename T>
  static T calc_yaw_difference(const T & yaw_1, const T & yaw_2);

  static double heading_from_quaternion(const geometry_msgs::msg::Quaternion q);
  static double heading_from_quaternion(const tf2::Quaternion q);
  static tf2::Quaternion quaternion_from_heading(const double & yaw);

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node & node_;
};
}  // namespace ckrobotics
#endif  // TRANSFORM_HELPER__TRANSFORM_HELPER_HPP_

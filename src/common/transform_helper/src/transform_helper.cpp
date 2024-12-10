#include <string>
#include <memory>

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "transform_helper/transform_helper.hpp"

namespace CKR
{
TransformHelper::TransformHelper(rclcpp::Node & node)
: node_(node)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

bool TransformHelper::lookup_transform(
  const std::string & source_frame, const std::string & target_frame,
  geometry_msgs::msg::TransformStamped & transform, const rclcpp::Time & time)
{
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, time);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_.get_logger(), "Could not transform %s to %s: %s",
      source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
  return true;
}

void TransformHelper::send_transform(const geometry_msgs::msg::TransformStamped & msg)
{
  tf_broadcaster_->sendTransform(msg);
}

template<>
double TransformHelper::calc_yaw_difference(const double & yaw_1, const double & yaw_2)
{
  tf2::Quaternion q1_inv;
  q1_inv.setRPY(0.0, 0.0, yaw_1);
  q1_inv.setW(q1_inv.getW() * -1.0);
  tf2::Quaternion q2;
  q2.setRPY(0.0, 0.0, yaw_2);
  const auto qr = q2 * q1_inv;
  tf2::Matrix3x3 m(qr);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double TransformHelper::heading_from_quaternion(const geometry_msgs::msg::Quaternion q)
{
  tf2::Quaternion q_tf2;
  tf2::fromMsg(q, q_tf2);
  return heading_from_quaternion(q_tf2);
}

double TransformHelper::heading_from_quaternion(const tf2::Quaternion q)
{
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

tf2::Quaternion TransformHelper::quaternion_from_heading(const double & yaw)
{
  auto q = tf2::Quaternion();
  q.setRPY(0.0, 0.0, yaw);
  return q;
}
}  // namespace CKR

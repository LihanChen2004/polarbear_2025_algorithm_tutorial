#include "euler_to_quaternion/euler_to_quaternion_node.hpp"

namespace euler_to_quaternion
{

EulerToQuaternion::EulerToQuaternion(const rclcpp::NodeOptions & options)
: Node("euler_to_quaternion", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting EulerToQuaternion!");

  euler_subscriber_ = this->create_subscription<euler_to_quaternion_interface::msg::MyEuler>(
    "euler", 10, std::bind(&EulerToQuaternion::eulerCallback, this, std::placeholders::_1));

  quaternion_publisher_ =
    this->create_publisher<euler_to_quaternion_interface::msg::MyQuaternion>("quaternion", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void EulerToQuaternion::eulerCallback(
  const euler_to_quaternion_interface::msg::MyEuler::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received Euler: r=%f, p=%f, y=%f", msg->roll, msg->pitch, msg->yaw);

  auto quaternion_msg = eulerToQuaternion(msg->roll, msg->pitch, msg->yaw);

  quaternion_publisher_->publish(quaternion_msg);

  geometry_msgs::msg::Quaternion ros_quaternion;
  ros_quaternion.x = quaternion_msg.x;
  ros_quaternion.y = quaternion_msg.y;
  ros_quaternion.z = quaternion_msg.z;
  ros_quaternion.w = quaternion_msg.w;

  geometry_msgs::msg::TransformStamped t;

  // map -> A
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "A";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation = ros_quaternion;
  tf_broadcaster_->sendTransform(t);

  // map -> B
  t.child_frame_id = "B";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 1.0;
  t.transform.translation.z = 2.0;
  tf_broadcaster_->sendTransform(t);
}

euler_to_quaternion_interface::msg::MyQuaternion EulerToQuaternion::eulerToQuaternion(
  double roll, double pitch, double yaw)
{
  euler_to_quaternion_interface::msg::MyQuaternion q;

  q.x =
    sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
  q.y =
    cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
  q.z =
    cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
  q.w =
    cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

  return q;
}
}  // namespace euler_to_quaternion

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(euler_to_quaternion::EulerToQuaternion)
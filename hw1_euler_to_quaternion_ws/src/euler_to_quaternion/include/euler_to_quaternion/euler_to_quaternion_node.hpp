#ifndef EULER_TO_QUATERNION__EULER_TO_QUATERNION_NODE_HPP
#define EULER_TO_QUATERNION__EULER_TO_QUATERNION_NODE_HPP

#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

#include "euler_to_quaternion_interface/msg/my_euler.hpp"
#include "euler_to_quaternion_interface/msg/my_quaternion.hpp"

namespace euler_to_quaternion
{
class EulerToQuaternion : public rclcpp::Node
{
public:
  /*
   * @brief EulerToQuaternion constructor
   * @param options Additional options to control creation of the node.
   */
  explicit EulerToQuaternion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /*
   * @brief Handle a new custom euler angle
   */
  void eulerCallback(const euler_to_quaternion_interface::msg::MyEuler::SharedPtr msg);

  /*
   * @brief Convert euler angles to quaternion
   * @param roll Roll angle in radians
   * @param pitch Pitch angle in radians
   * @param yaw Yaw angle in radians
   * @return Quaternion message
   */
  euler_to_quaternion_interface::msg::MyQuaternion eulerToQuaternion(
    double roll, double pitch, double yaw);

  // Broadcast tf from odom to A/B
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<euler_to_quaternion_interface::msg::MyEuler>::SharedPtr euler_subscriber_;
  rclcpp::Publisher<euler_to_quaternion_interface::msg::MyQuaternion>::SharedPtr
    quaternion_publisher_;
};
}  // namespace euler_to_quaternion

#endif  // EULER_TO_QUATERNION__EULER_TO_QUATERNION_NODE_HPP

#include "rt_fcs_tf2/FcsTf2Node.h"

#include <functional>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rt_fcs {

FcsTf2Node::FcsTf2Node(const rclcpp::NodeOptions &options)
    : Node("rt_fcs_tf2", options) {
  RCLCPP_WARN(get_logger(), "Start rt_fcs::tf2_node!");

  _tfBroadCaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  _subGimbalInfo = this->create_subscription<rt_interfaces::msg::GimbalInfo>(
      "/gimbal/info", rclcpp::SensorDataQoS{},
      std::bind(&FcsTf2Node::gimbalInfoCallBack, this, std::placeholders::_1));
}

void FcsTf2Node::gimbalInfoCallBack(const rt_interfaces::msg::GimbalInfo &msg) {
  geometry_msgs::msg::TransformStamped t;
  double timestampOffset = 0.0;

  t.header.stamp =
      this->now() + rclcpp::Duration::from_seconds(timestampOffset);
  t.header.frame_id = "odom";
  t.child_frame_id = "gimbal_link";

  tf2::Quaternion q;
  q.setRPY(msg.roll, msg.pitch, msg.yaw);
  t.transform.rotation = tf2::toMsg(q);
  _tfBroadCaster->sendTransform(t);
}

} // namespace rt_fcs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rt_fcs::FcsTf2Node)

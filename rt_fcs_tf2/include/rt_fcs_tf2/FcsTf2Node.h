#ifndef RT_FCS__RT_FCS_TF2_NODE
#define RT_FCS__RT_FCS_TF2_NODE

#include "rt_interfaces/msg/gimbal_info.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace rt_fcs {

class FcsTf2Node : public rclcpp::Node {
public:
  explicit FcsTf2Node(const rclcpp::NodeOptions &options);

private:
  void gimbalInfoCallBack(const rt_interfaces::msg::GimbalInfo &msg);

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tfBroadCaster;

  rclcpp::Subscription<rt_interfaces::msg::GimbalInfo>::SharedPtr _subGimbalInfo;
};

} // namespace rt_fcs

#endif
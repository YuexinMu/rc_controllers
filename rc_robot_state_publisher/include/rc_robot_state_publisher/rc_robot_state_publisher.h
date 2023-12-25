//
// Created by myx on 2023/3/30.
//
// ref:https://github.com/rm-controls

#pragma once

#include <rc_common/hardware_interface/robot_state_interface.h>
#include <rc_common/tf_rt_broadcaster.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace rc_robot_state_publisher
{
class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, std::string p_root, std::string p_tip)
    : segment(p_segment), root(std::move(p_root)), tip(std::move(p_tip))
  {
  }

  KDL::Segment segment{};
  std::string root, tip;
};

class RcRobotStatePublisher
  : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                                          rc_control::RobotStateInterface>
{
public:
  RcRobotStatePublisher() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;

private:
  void addChildren(KDL::SegmentMap::const_iterator segment);
  void tfSubCallback(const tf2_msgs::TFMessageConstPtr& msg);
  void staticSubCallback(const tf2_msgs::TFMessageConstPtr& msg);

  urdf::Model model_{};
  std::map<std::string, urdf::JointMimicSharedPtr>* mimic_{};
  unsigned int num_hw_joints_{};
  bool use_tf_static_{};
  bool ignore_timestamp_{};
  double publish_rate_{};
  ros::Time last_update_;
  ros::Time last_publish_time_;

  std::map<std::string, hardware_interface::JointStateHandle> jnt_states_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;

  tf2_ros::Buffer* tf_buffer_{};
  rc_common::TfRtBroadcaster tf_broadcaster_;
  rc_common::StaticTfRtBroadcaster static_tf_broadcaster_;
  // Do not use tf2_ros::TransformListener because it will lead to setTransform calling twice when publishing the transform
  ros::Subscriber tf_sub_;
  ros::Subscriber tf_static_sub_;
  realtime_tools::RealtimeBuffer<tf2_msgs::TFMessage> tf_msg_;
  realtime_tools::RealtimeBuffer<tf2_msgs::TFMessage> tf_static_msg_;
};

}  // namespace rc_robot_state_publisher

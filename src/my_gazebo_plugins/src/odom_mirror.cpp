#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo {
class OdomMirror : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    model_ = _model;
    node_ = gazebo_ros::Node::Get(_sdf);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Update model pose from odometry
        ignition::math::Pose3d pose(
          msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          msg->pose.pose.position.z,
          msg->pose.pose.orientation.w,
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z
        );
        model_->SetWorldPose(pose);
      });
  }

private:
  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
GZ_REGISTER_MODEL_PLUGIN(OdomMirror)
} // namespace gazebo

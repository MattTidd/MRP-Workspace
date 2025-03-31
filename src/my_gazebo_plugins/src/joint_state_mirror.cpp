#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace gazebo {
class JointStateMirror : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    node_ = gazebo_ros::Node::Get(_sdf);
    model_ = _model;
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&JointStateMirror::OnJointState, this, std::placeholders::_1));
  }

private:
  void OnJointState(const sensor_msgs::msg::JointState::SharedPtr _msg) {
    // RCLCPP_INFO(node_->get_logger(), "received joint states (count: %zu)", _msg->name.size());
    for (size_t i = 0; i < _msg->name.size(); ++i) {
      auto joint = model_->GetJoint(_msg->name[i]);
      if (joint) {
        joint->SetPosition(0, _msg->position[i]);
        RCLCPP_DEBUG(node_->get_logger(), "Set joint '%s' to %f", 
          _msg->name[i].c_str(), _msg->position[i]);
      }
    }
  }

  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};
GZ_REGISTER_MODEL_PLUGIN(JointStateMirror)
} // namespace gazebo
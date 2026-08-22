#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pendulum_interfaces/msg/pendulum_state.hpp"

class JointStateBridge : public rclcpp::Node
{
public:
  JointStateBridge()
  : Node("joint_state_bridge")
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    state_sub_ = this->create_subscription<pendulum_interfaces::msg::PendulumState>("pendulum_state", 10,
      std::bind(&JointStateBridge::state_cb, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<pendulum_interfaces::msg::PendulumState>::SharedPtr state_sub_; 

  void state_cb(const pendulum_interfaces::msg::PendulumState::SharedPtr msg)
  {
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name.push_back("rail_to_cart");
    joint_state.name.push_back("cart_to_pole");
    joint_state.header.stamp = this->now();
    joint_state.position.push_back(msg->cart_position);
    joint_state.position.push_back(msg->pendulum_angle);
    
    joint_pub_ ->publish(joint_state);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateBridge>());
  rclcpp::shutdown();
  return 0;
}

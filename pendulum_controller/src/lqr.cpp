#include "rclcpp/rclcpp.hpp"
#include "pendulum_interfaces/msg/pendulum_state.hpp"
#include "pendulum_interfaces/msg/pendulum_command.hpp"

constexpr double K_x = -1.0;
constexpr double K_x_dot = -2.1196;
constexpr double K_theta = -32.3382;
constexpr double K_theta_dot = -6.7134;

class PendulumController : public rclcpp::Node{
public:
  PendulumController()
  : Node("pendulum_controller")
  {
    command_pub_ =
      this->create_publisher<pendulum_interfaces::msg::PendulumCommand>("pendulum_command", 10);
    state_sub_ =
      this->create_subscription<pendulum_interfaces::msg::PendulumState>("pendulum_state", 10,
        std::bind(&PendulumController::state_cb, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<pendulum_interfaces::msg::PendulumCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<pendulum_interfaces::msg::PendulumState>::SharedPtr state_sub_;
  void state_cb(const pendulum_interfaces::msg::PendulumState::SharedPtr msg)
  {
    double x_ = msg->cart_position;
    double x_dot_ = msg->cart_velocity;
    double theta_ = msg->pendulum_angle;
    double theta_dot_ = msg->pendulum_angular_velocity;
    double force_ = -(K_x * x_ + K_x_dot * x_dot_ + K_theta * theta_ + K_theta_dot * theta_dot_);

    auto command_msg = pendulum_interfaces::msg::PendulumCommand();
    command_msg.header.stamp = this->now();
    command_msg.cart_force = force_;
    command_pub_->publish(command_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumController>());
  rclcpp::shutdown();
}

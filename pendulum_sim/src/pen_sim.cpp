#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "pendulum_interfaces/msg/pendulum_state.hpp"
#include "pendulum_interfaces/msg/pendulum_command.hpp"
#include "pendulum_sim/sim_math.hpp"

using namespace std::chrono_literals;

class PendulumSim : public rclcpp::Node
{
public:
  PendulumSim()
  : Node("pendulum_sim")
  {
    RCLCPP_INFO(this->get_logger(), "pendulum sim started");
    state_pub_ = this->create_publisher<pendulum_interfaces::msg::PendulumState>("pendulum_state",
      10);
    command_sub_ = this->create_subscription<pendulum_interfaces::msg::PendulumCommand>(
        "pendulum_command", 10, std::bind(&PendulumSim::command_cb, this, std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(1ms, std::bind(&PendulumSim::timer_cb, this));
  }

private:
  State state_ = {0.0, 0.0, 0.5, 0.0};
  double force_ = 0.0;

  rclcpp::Publisher<pendulum_interfaces::msg::PendulumState>::SharedPtr state_pub_;
  rclcpp::Subscription<pendulum_interfaces::msg::PendulumCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void command_cb(const pendulum_interfaces::msg::PendulumCommand::SharedPtr msg)
  {
    force_ = msg->cart_force;
  }

  void timer_cb()
  {
    constexpr double dt = 0.001;
    state_ = rk4step(state_, force_, dt);
    auto msg = pendulum_interfaces::msg::PendulumState();
    msg.header.stamp = this->now();
    msg.cart_position = state_.x;
    msg.cart_velocity = state_.x_dot;
    msg.pendulum_angle = state_.theta;
    msg.pendulum_angular_velocity = state_.theta_dot;

    state_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumSim>());
  rclcpp::shutdown();
  return 0;
}

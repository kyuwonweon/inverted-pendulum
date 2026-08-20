#include <rclcpp/rclcpp.hpp>
#include "pendulum_interfaces/msg/pendulum_state.hpp"
#include "pendulum_interfaces/msg/pendulum_command.hpp"

using namespace std::chrono_literals;

constexpr double M  = 1.0;  // cart (kg)
constexpr double m = 0.1;  // pole (kg)
constexpr double l = 0.5;  // pole com (m)
constexpr double g = 9.81;  // gravity

struct State {
  double x;
  double x_dot;
  double theta;
  double theta_dot;
};

class PendulumSim : public rclcpp::Node
{
  public:
    PendulumSim()
    : Node("pendulum_sim")
    {
      RCLCPP_INFO(this->get_logger(), "pendulum sim started");
      state_pub_ = this-> create_publisher<pendulum_interfaces::msg::PendulumState>("pendulum_state", 10);
      command_sub_ = this-> create_subscription<pendulum_interfaces::msg::PendulumCommand>(
        "pendulum_command", 10, std::bind(&PendulumSim::command_cb, this, std::placeholders::_1)
      );
      timer_ = this->create_wall_timer(1ms, std::bind(&PendulumSim::timer_cb, this));
    }

  private:
    State state_ = {0.0, 0.0, 0.15, 0.0};

    rclcpp::Publisher<pendulum_interfaces::msg::PendulumState>::SharedPtr state_pub_;
    rclcpp::Subscription<pendulum_interfaces::msg::PendulumCommand>::SharedPtr command_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void command_cb(const pendulum_interfaces::msg::PendulumCommand::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "recieved force: %f", msg->cart_force);
    }

    void timer_cb(){
      RCLCPP_INFO(this->get_logger(), "add details later");
    }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumSim>());
  rclcpp::shutdown();
  return 0;
}
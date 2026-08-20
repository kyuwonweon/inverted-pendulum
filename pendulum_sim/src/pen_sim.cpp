#include <rclcpp/rclcpp.hpp>

class PendulumSim : public rclcpp::Node
{
  public:
    PendulumSim()
    : Node("pendulum_sim")
    {
      RCLCPP_INFO(this->get_logger(), "pendulum sim started");
    }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumSim>());
  rclcpp::shutdown();
  return 0;
}
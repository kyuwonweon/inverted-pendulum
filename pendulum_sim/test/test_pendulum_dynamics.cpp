#include <gtest/gtest.h>
#include "pendulum_sim/sim_math.hpp"

TEST(PendulumDynamics, Stationary)
{
  State s{0.0, 0.0, 0.0, 0.0};

  State d = derivative(s, 0.0);

  EXPECT_NEAR(d.x_dot, 0.0, 1e-9);
  EXPECT_NEAR(d.theta_dot, 0.0, 1e-9);
}

TEST(PendulumDynamics, Force_Applied)
{
  State s{0.0, 0.0, 0.0, 0.0};
  State d = derivative(s, 1.0);
  EXPECT_NEAR(d.theta_dot, -2.0, 1e-9);
}

TEST(PendulumDynamics, Directionaltiy_of_angle)
{
  State s{0.0, 0.0, 0.05, 0.0};
  State d = derivative(s, 0.0);
  EXPECT_GT(d.theta_dot, 0.0);
}

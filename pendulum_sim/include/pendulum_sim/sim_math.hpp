#pragma once
#include <cmath>


constexpr double M = 1.0;   // cart (kg)
constexpr double m = 0.1;  // pole (kg)
constexpr double l = 0.5;  // pole com (m)
constexpr double g = 9.81;  // gravity

struct State
{
  double x;
  double x_dot;
  double theta;
  double theta_dot;
};

inline State operator+(const State & a, const State & b)
{
  return State{a.x + b.x, a.x_dot + b.x_dot, a.theta + b.theta, a.theta_dot + b.theta_dot};
}

inline State operator*(const State & s, double scalar)
{
  return State{s.x * scalar, s.x_dot * scalar, s.theta * scalar, s.theta_dot * scalar};
}

inline State derivative(const State & s, double force)
{
  double x_ddot = (force + m * l * s.theta_dot * s.theta_dot * std::sin(s.theta) -
    m * g * std::sin(s.theta) * std::cos(s.theta)) /
    (M + m * std::sin(s.theta) * std::sin(s.theta));
  double theta_ddot = ((M + m) * g * std::sin(s.theta) - force * std::cos(s.theta) -
    m * l * s.theta_dot * s.theta_dot * std::sin(s.theta) * std::cos(s.theta)) /
    (l * (M + m * std::sin(s.theta) * std::sin(s.theta)));
  return State{s.x_dot, x_ddot, s.theta_dot, theta_ddot};
}

inline State rk4step(const State & s, double force, double dt)
{
  State k1 = derivative(s, force);
  State k2 = derivative(s + k1 * (dt / 2.0), force);
  State k3 = derivative(s + k2 * (dt / 2.0), force);
  State k4 = derivative(s + k3 * dt, force);
  return s + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}

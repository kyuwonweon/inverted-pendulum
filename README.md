**Inverted Pendulum System Modeling and Control**

Kyuwon Weon

ROS2/C++ simulation and control of a cart-pole (inverted pendulum) system:
nonlinear dynamics integrated with RK4, stabilized at the upright equilibrium
with an LQR controller, visualized live in RViz.

**Packages**

- `pendulum_interfaces` — custom messages: `PendulumState` (cart position/velocity,
  pole angle/angular velocity), `PendulumCommand` (cart force).
- `pendulum_sim` — integrates the nonlinear cart-pole dynamics (RK4) given the
  commanded force, publishes `PendulumState`.
- `pendulum_controller` — LQR controller; computes cart force from the current
  state, publishes `PendulumCommand`.
- `pendulum_description` — URDF model, a `joint_state_bridge` node (translates
  `PendulumState` into `sensor_msgs/JointState`), RViz config, and the launch
  file tying everything together for visualization.

**Topology**

```
pendulum_sim --PendulumState--> pendulum_controller --PendulumCommand--> pendulum_sim   (control loop)
pendulum_sim --PendulumState--> joint_state_bridge --JointState--> robot_state_publisher --TF--> rviz2
```

**Build & run**

```
colcon build
source install/setup.bash
ros2 launch pendulum_description display.launch.py
```
# gazebo-torque-calculation-bug

Small example repository that showcased a gazebo torque bug that was discussed in [frankaemika/franka_ros#160](https://github.com/frankaemika/franka_ros/issues/160#issuecomment-961780423). To summarize the [gazebo::physics::ODEJoint::GetForceTorque()](https://github.com/osrf/gazebo/blob/gazebo9/gazebo/physics/ode/ODEJoint.cc#L651-L849) calculates the wrong torque when the joint orgin is rotated.

## Installation

First make sure you have all the required dependencies installed by executing the following command:

```bash
rosdep install --from-path src --ignore-src -r -y
```

Following install the [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) package and build the catkin workspace in debug mode:

```bash
catkin build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

You can ofcourse also use `catkin_make`. The `DCMAKE_EXPORT_COMPILE_COMMANDS` is only neede when you want [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) support.

## Use

Run the following command to see the working example

```bash
roslaunch gazebo_torque_calculation_bug stick.launch
```

You can then step through the simulation to see the torque printed. After you inspected this version you can
run the following command to see the example in which the `GetForceTorque()` function calculates the wrong torque:

```bash
roslaunch gazebo_torque_calculation_bug stick.launch rotate_aor:=true
```

## Bug report

The full bug report can be found [here](https://github.com/frankaemika/franka_ros/issues/160#issuecomment-961780423).

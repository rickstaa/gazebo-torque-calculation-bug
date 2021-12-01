# gazebo-torque-calculation-bug

Small example repository that showcased a Gazebo torque bug that was discussed in [frankaemika/franka_ros#160](https://github.com/frankaemika/franka_ros/issues/160#issuecomment-961780423). To summarize the [gazebo::physics::ODEJoint::GetForceTorque()](https://github.com/osrf/gazebo/blob/gazebo9/gazebo/physics/ode/ODEJoint.cc#L651-L849) calculates the wrong torque when the joint orgin is rotated.

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

## Gazebo example instructions

There are two modes in which you can see the bug the first mode is when `load_several_sticks` is `false`. In this you can see the working example using the following command:

```bash
roslaunch gazebo_torque_calculation_bug stick.launch load_several_sticks:=false
```

You can then step through the simulation to see the torques in the terminal and the plot. After you inspected this version you can
run the following command to see the example in which the `GetForceTorque()` function calculates the wrong torque:

```bash
roslaunch gazebo_torque_calculation_bug stick.launch load_several_sticks:=false rotated:=true initial_joint_positions:='-J stick_joint1 1.57079632679'
```

Alternatively, you can see both in action when `load_several_sticks` is `true`:

```bash
roslaunch gazebo_torque_calculation_bug stick.launch load_several_sticks:=true
```

In this:

- `stick`: Is the normal working example.
- `stick2`: Same as above but now the joint is rotated by 180 degrees.
- `stick3`: The fliped (WRONG) example.
- `stick4`: Same as above but now the joint is rotated by 180 degrees.

In both options you can control the stick joints using the `rqt_joint_trajectory_controller` windows.

### Parameters

The script contains the following ROS parameters:

The script contains the following ROS parameters:

- `log_bug_info`: Enable/disable bug console logs.
- `sparse_bug_info`: Only show debug info for first joint.

### Topics

The bug info that is printed to the console is also published under the `/<STICK_NAME>/gazebo_bug` topic. The error is the difference between the calculated effort and gravity torque. The distinction between `error` and `error2` lies in the fact that the `error` is calculated as follows:

```cpp
auto gazebo_effort = Eigen::Vector3d(
      gazebo_torque.X(),
      gazebo_torque.Y(),
      gazebo_torque.Z()
    ).dot(urdf_axis);
```

where error2 uses the effort that comes from the gazebos [GetForce](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Joint.html#aebc39094623208f497a38b91cc51f7fe) method.

## Ignition Gazebo example

You can also check whether this bug is present in [ignition 6.0](https://ignitionrobotics.org/home). Use the following command to start the nominal case:

```bash
roslaunch gazebo_torque_calculation_bug stick_ignition.launch
```

To start the bugged case use:

```bash
roslaunch gazebo_torque_calculation_bug stick_ignition.launch rotated:=true
```

After you started the simulation you can check the joint force and torque by using the following command:

```bash
ign topic -e -n1 -t /stick_joint1/force_torque
```

In both options you can control the stick joints using the `Joint position controller` GUI.

## Bug report

The full bug report can be found [here](https://github.com/frankaemika/franka_ros/issues/160#issuecomment-961780423).

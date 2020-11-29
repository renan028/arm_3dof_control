# RemyRobotControl

This package provides a simple Kinematic Control for a 3-DOF (only revolute joints)
manipulator with the following DH parameters:

| Link  | Twist | Length | Offset | angle |
| ----- | ----- | ------ | ------ | ----- |
|   0   |   0   |   0    |   -    |   -   |
|   1   |  pi/2 |   10   |   -    |   t1  |
|   2   |   0   |   5    |   -    |   t2  |
|   3   |   0   |   5    |   -    |   t3  |

And the following joints' limits:
| Joint |  Min  |  Max  |
| ----- | ----- | ----- |
|   1   |  -pi  |   pi  |
|   2   | -pi/2 | pi/2  |
|   3   |  -pi  |   pi  |

<img src="https://raw.githubusercontent.com/renan028/robot_control/master/images/robot.png" width="400">
---
<br />

## [Robot](https://github.com/renan028/robot_control/blob/master/include/remy_robot_control/robot.h)
---
The [Robot](https://renan028.github.io/robot_control/classremy__robot__control_1_1Robot.html) 
class provides forward and inverse kinematics for the given manipulator. 

There are three implemented solutions for the inverse kinematics problem:
* [Analytical](https://renan028.github.io/robot_control/classremy__robot__control_1_1Robot.html#a1120c7173f56ff5293bc29604c9d2c37)
* [Damped Least Square](https://renan028.github.io/robot_control/classremy__robot__control_1_1Robot.html#ae0f7f33fa2f5bc4cdd83f2c2c97db683)
* [Jacobian Transpose](https://renan028.github.io/robot_control/classremy__robot__control_1_1Robot.html#afc2192f6be3c5e0407632f218b9fb5d3)

There are two implemented solutions for the forward kinematics problem:
* [Generic](https://renan028.github.io/robot_control/classremy__robot__control_1_1Robot.html#ab8a09d0556766fd8ba041091c7f5c1cc) solution
* [Fast](https://renan028.github.io/robot_control/classremy__robot__control_1_1Robot.html#a65d08160e5c0f91011014dd6f2694e0b) (project specific) solution

Unittests for all [IK](https://github.com/renan028/robot_control/blob/master/tests/test_ik.h) and [FK](https://github.com/renan028/robot_control/blob/master/tests/test_fk.h) are available. 

All equations and math for each solution are documented in the source code.
<br />
It is important to note that the code is flexible enough to integrate new solutions,
because it explores the function object wrapper concept with [lambda functions](https://renan028.github.io/robot_control/classremy__robot__control_1_1Robot.html#a2daad9df3ede21e8fdb2ff7f72f5c97a).

The Robot's settings can also be changed at runtime, by using the custom [RemyRobotSettings](https://renan028.github.io/robot_control/structremy__robot__control_1_1RemyRobotSettings.html) struct.

A custom class [Angles](https://renan028.github.io/robot_control/classremy__robot__control_1_1Angle.html)
was implemented to deal with angle limits and to properly wrap an angle between [-pi,pi]. Therefore, the robot uses the custom [RemyJoints](https://github.com/renan028/robot_control/blob/master/include/remy_robot_control/types.h#L40) struct, and joints manipulation are transparent. One may see the 
[unittests](https://github.com/renan028/robot_control/blob/master/tests/test_angle.h) to fully understand.

<br />

## [Control](https://github.com/renan028/robot_control/blob/master/include/remy_robot_control/control.h)
---
The [Control](https://renan028.github.io/robot_control/classremy__robot__control_1_1Control.html) has a model of the Robot and it provides Kinematic Control. There are two implemented solutions for 
the Kinematic Control:
* [Proportional + Feedforward with DLS](https://renan028.github.io/robot_control/classremy__robot__control_1_1Control.html#ac540f0e69b0754df81d9f8a28c82c01d), which is a more generic control but requires some parameter tuning. It also has an extra term to avoid singularities (which is the case). Math is explained in the source file.
* [Analytical](https://renan028.github.io/robot_control/classremy__robot__control_1_1Control.html#a92381b3009f3c3258318833500aa3a1a), a project specific solution which uses the exact inverse kinematics.

As in the Robot file, the concept of function object wrapper was explored for flexibility.

The Control settings can the changed with the custom [RemyControlSettings](https://renan028.github.io/robot_control/structremy__robot__control_1_1RemyControlSettings.html) struct. The minimum value (50) for the control's frequency is verified.

The Control's constructor requires a string argument, which is the absolute path to the input (waypoints) file. It has a method to parse that file.

The Control has a **shared_ptr** to **connection**. In this particular setup and example, that *shared_ptr\<connection>* is shared (as a *weak_ptr*) to the RoboticSystem for communication between threads. 

> **_Note:_**  In this architecture, I am using threads and sharing resources through the **connection** class. An inter-process system would be a more real/generic approach here, but it is not in the scope of this task. You don't need to use ROS, however, and one of the faster solutions for a two publisher/subscriber architecture would be to use ZMQ, as already implemented by me in this other project [here](https://github.com/MiRON-project/ZMQServer).  

The **start()/stop()** methods initializes/pauses the communication with the RoboticSystem (sends/receives data through *connections*). For testing purposes, the **start(weak_ptr\<connection>)** method requires the **connection** pointer of the RoboticSystem (see below in the RoboticSystem section). The [start](https://github.com/renan028/robot_control/blob/master/src/control.cc#L69) loop works as following:

1. Check if connection is open and the resource is available (*weak_ptr*)
2. Receive the joints values and [convert](https://github.com/renan028/robot_control/blob/master/include/remy_robot_control/utils.h#L30) it to float by pointer cast. The unittests of signal conversion are available [here](https://github.com/renan028/robot_control/blob/master/tests/test_data_convert.h)
3. Compute new control signal
4. Convert control signal to vector\<char>
5. Send control signal

The Controller frequency is 50Hz.

For better understanding, the Control Unittests are available [here](https://github.com/renan028/robot_control/blob/master/tests/test_control.cc).

### [Trajectory](https://github.com/renan028/robot_control/blob/master/include/remy_robot_control/trajectory.h)

The waypoints are fed in the Controller, which generates a [trajectory](https://renan028.github.io/robot_control/classTrajectory.html), a continuous and feasible path (cartesian coordinates for the end-effector) over time. A piecewise-linear interpolation of the points was chosen, then a constant velocity in cartesian space is acquired. 

> **_Note:_** The standard choice (and actually the choice for production) would be a 3DSpline composed of a third/fourth polynomial degree (at least continuous accelerations). It would be possible to evaluate and constraint torques. Since we are dealing with kinematics and no saturation, we kept it simple.

The **update(t)** method calculates the desired position/velocity for the end-effector. Unitests are available [here](https://github.com/renan028/robot_control/blob/master/tests/test_trajectory.h).

<br />

## [RoboticSystem](https://github.com/renan028/robot_control/blob/master/include/remy_robot_control/robot_system.h)
---

The [RoboticSystem](https://renan028.github.io/robot_control/classremy__robot__control_1_1RobotSystem.html) mocks the robot (it has a Robot class), encoders, and it communicates with the Controller through **connection**. The System has it own parameters [RemySystemSetting](https://renan028.github.io/robot_control/structremy__robot__control_1_1RemySystemSettings.html).

To start communication, one must pass the shared communication resource **connection**. See the [test_integration](https://github.com/renan028/robot_control/blob/master/tests/test_integration.cc). The main communication loop is available at [start](https://github.com/renan028/robot_control/blob/master/src/robot_system.cc#L59), and it works as following:

1. Check if connection is open and the resource is available (*weak_ptr*)
2. Move the manipulator with the last control input by the elapsed time
3. Get the new Robot joints with FK
4. Mock the [Encoder](https://github.com/renan028/robot_control/blob/master/src/utils.cc#L68) precision lost
5. [Convert](https://github.com/renan028/robot_control/blob/master/src/utils.cc#L24) the joints value to vector\<char> with pointer cast
6. Send the data through **connection**
7. Receive the control signal

The frequency is 1000 Hz.

<br />

## [Configuration](https://github.com/renan028/robot_control/blob/master/include/remy_robot_control/config/config.json)
---

The Configuration is a JSON file and it is read in the main file to set Control, Robot and System parameters.

<br />

## 3rdParty and Dependencies
---
The system uses the header-only file json.hpp for json parser. It also uses GTest for unittests and Eigen (Required) for all math.

<br />

## Installation
---

To install and run the system, clone this repository and run, in the root folder:
```
mkdir build
cd build
cmake ..
make
```
Then, in the build folder, run (the two arguments are required):
```
./RemyRobotControl <path_to_input> <path_to_config>
```
The output file "out.csv" is composed of time, the end-effector path (x,y,z), the joints path (theta1, theta2, theta3) and control sign for each joint.

To run unittests, one just need to run them individually, for instance:
```
./RemyRobotControl_Test
```

<br />

## Results
---

The output, available in the "result" folder represents the following images:


<img src="https://raw.githubusercontent.com/renan028/robot_control/master/images/path.png" width="480">
<img src="https://raw.githubusercontent.com/renan028/robot_control/master/images/xyz.png" width="480">
<img src="https://raw.githubusercontent.com/renan028/robot_control/master/images/joints.png" width="480">
<img src="https://raw.githubusercontent.com/renan028/robot_control/master/images/control.png" width="480">



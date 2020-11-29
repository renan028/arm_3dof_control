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

![robot](images/robot.png | height=100)

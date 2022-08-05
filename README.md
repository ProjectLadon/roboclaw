# Roboclaw
Roboclaw is an extensible series of [Roboclaw][roboclaw] nodes for [ROS2][ros2]

## Features

- The base node "roboclaw_node" supports up to 8 Roboclaw controllers using packet serial mode
- Drive systems and odometry are decoupled from the base Roboclaw node
- A differential drive node is supported out of the box
- Written in roscpp for effecient memory usage and performance

## Requirements
- ROS2 Humble

## Nodes

### roboclaw_node

#### Parameters

| Param | Type  | Description  |
| :------------- |:-------------| :-----|
| serial_port | string | Path to the serial port to use |
| baudrate | int | Baudrate of the serial port |
| num_claws | int | Number of Roboclaw controllers in packet serial mode |
| timeout_ms | int | Serial timeout, in milliseconds |

#### Topics
| Action | Topic | Type |
| :------------- |:-------------| :-----|
| publish | ~/claw{idx}/posn_out | roboclaw/msg/EncoderSteps |
| publish | ~/claw{idx}/volts_amps_out | roboclaw/msg/MotorVoltsAmps |
| publish | ~/claw{idx}/velocity_out | roboclaw/msg/EncoderVelocity |
| subscribe | ~/claw{idx}/motor_vel_cmd | roboclaw/msg/MotorVelocity |
| subscribe | ~/claw{idx}/motor_vel_single_cmd | roboclaw/msg/MotorVelocitySingle |
| subscribe | ~/claw{idx}/motor_pos_cmd | roboclaw/msg/MotorPosition |
| subscribe | ~/claw{idx}/motor_pos_single_cmd | roboclaw/msg/MotorPositionSingle |
| subscribe | ~/claw{idx}/motor_duty_single_cmd | roboclaw/msg/MotorDutySingle |

#### Notes

- {idx} is the index of the target Roboclaw controller
- When using one Roboclaw controller, configure it in packet serial mode with address 0x80. This will be motor index 0 in RoboclawEncoderSteps and RoboclawMotorVelocity.
- When using more than one Roboclaw controller, configure them in packet serial mode with 0x80 being motor index 0, 0x81 being motor index 1, and so on.

### diffdrive_node

#### This segment has not yet been updated for ROS2

#### Parameters

| Param | Type  | Description  |
| :------------- |:-------------| :-----|
| steps_per_meter | string | Number of encoder steps per meter |
| base_width | int | Diameter of the robots base from the center of each wheel |
| swap_motors | bool | Swap motor1 with motor2
| invert_motor_1 | bool | Invert drive and odometry for motor1
| invert_motor_2 | bool | Invert drive and odometry for motor2

#### Topics
| Action | Topic | Type |
| :------------- |:-------------| :-----|
| publish | odom | Odometry |
| subscribe | cmd_vel | Twist |

## Planned

- Support for the [NASA JPL Open Source Rover][jpl]'s drive system
- Exposing more of the Roboclaw's functionality via messages / services



[roboclaw]: http://www.basicmicro.com
[ros]: http://www.ros.org
[jpl]: https://opensourcerover.jpl.nasa.gov
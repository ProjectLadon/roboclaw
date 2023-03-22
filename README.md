# Roboclaw
Roboclaw is an extensible series of [Roboclaw][roboclaw] nodes for [ROS2][ros2]

## Features

- The base node "roboclaw_node" supports up to 8 Roboclaw controllers using packet serial mode
- Drive systems and odometry are decoupled from the base Roboclaw node
- A differential drive node is supported out of the box
- Written in roscpp for effecient memory usage and performance

## Requirements
- [ROS2 Humble Hawksbill][ros2]
- Boost

It may work with other ROS2 releases, but it has only been tested with Humble Hawksbill

## Nodes

### roboclaw_node

This node controls one or more Roboclaw motor controllers communicating in packet serial mode. For more information on the behavior of the Roboclaws and the concepts referenced below, see the [user manual][https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf].

#### Parameters

| Param | Type  | Description  | Settable while Running? | Notes |
| :------------- |:-------------| :-----| :-----| :-----|
| ``serial_port`` | string | Path to the serial port to use | No |
| ``baudrate`` | int | Baudrate of the serial port | No |
| ``num_claws`` | int | Number of Roboclaw controllers in packet serial mode | No |
| ``timeout_ms`` | int | Serial timeout, in milliseconds | No |
| ``statistics_enable`` | bool | Collect statistics on subscribed topics | No |
| ``velocity_hz`` | float | Frequency at which to collect and report velocity from all Roboclaws | No | A value of less than zero turns off velocity reporting |
| ``volt_amp_hz`` | float | Frequency at which to collect and report voltage and current data from all Roboclaws | No | A value of less than zero turns off voltage and current reporting |
| ``status_hz`` | float | Frequency at which to collect and report status data from all Roboclaws | No | A value of less than zero turns off status reporting |
| ``pwm_hz`` | float | Frequency at which to collect and report PWM data from all Roboclaws | No | A value of less than zero turns off PWM reporting |
| ``posn_hz`` | float | Frequency at which to collect and report position data from all Roboclaws | No | A value of less than zero turns off position reporting |
| ``posn_err_hz`` | float | Frequency at which to collect and report position error data from all Roboclaws | No | A value of less than zero turns off position error reporting |
| ``current_limit.node{idx}.chan{1|2}.enable`` | bool | Enable setting of current limit on the given roboclaw and channel. | No | Turning this on allows setting of the roboclaw's internal current limit. |
| ``current_limit.node{idx}.chan{1|2}.limit`` | float | Value to set the roboclaw current limit to. | Yes | Changing this parameter will immediately update the roboclaw's internal current limit |
| ``enable.node{idx}.duty.chan{1|2}`` | bool | Subscribe to duty cycle commands for the given Roboclaw and channel | No | This exists mostly to clean up the subscriptions that would otherwise be generated |
| ``enable.node{idx}.position.chan{1|2}`` | bool | Subscribe to position commands for the given Roboclaw and channel | No | This exists mostly to clean up the subscriptions that would otherwise be generated |
| ``enable.node{idx}.position.both`` | bool | Subscribe to dual position commands for the given Roboclaw | No | This exists mostly to clean up the subscriptions that would otherwise be generated |
| ``enable.node{idx}.velocity.chan{1|2}`` | bool | Subscribe to velocity commands for the given Roboclaw and channel | No | This exists mostly to clean up the subscriptions that would otherwise be generated |
| ``enable.node{idx}.velocity.both`` | bool | Subscribe to dual velocity commands for the given Roboclaw | No | This exists mostly to clean up the subscriptions that would otherwise be generated |
| ``posn_limits.node{idx}.chan{1|2}.center`` | int | Unused (due to be eliminated)
| ``posn_limits.node{idx}.chan{1|2}.lower_limit`` | int | Position commands larger than this value will be bounded to it | No | 
| ``posn_limits.node{idx}.chan{1|2}.upper_limit`` | int | Position commands small  than this value will be bounded to it | No | 
| ``rate_limits.node{idx}.chan{1|2}`` | int | Rate divisor for incoming commands to the given Roboclaw and channel. For example, a value of 1 will lead to only every other command will be executed. | No | Poorly named and implemented; should be changed to a max frequency for a given command. |
| ``pid.node{idx}.chan{1|2}.pos.enable`` | bool | Enable setting of roboclaw onboard position PID constants for the given roboclaw and channel via parameter | No | If this is set to true, the PID constants will be set on startup to whatever the parameters are. | 
| ``pid.node{idx}.chan{1|2}.pos.p`` | float | Position proportional constant | Yes | Changing this parameter causes all position PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.pos.i`` | float | Position integral constant | Yes | Changing this parameter causes all position PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.pos.max_i`` | float | Position integral wind-up limit | Yes | Changing this parameter causes all position PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.pos.d`` | float | Position derivative constant | Yes | Changing this parameter causes all position PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.pos.max`` | int | Position maximum value | Yes | Changing this parameter causes all position PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.pos.min`` | int | Position minimum value | Yes | Changing this parameter causes all position PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.pos.deadzone`` | int | Position deadzone | Yes | Changing this parameter causes all position PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.vel.enable`` | bool | Enable setting of roboclaw onboard velocity PID constants for the given roboclaw and channel via parameter | No | If this is set to true, the PID constants will be set on startup to whatever the parameters are. |
| ``pid.node{idx}.chan{1|2}.vel.p`` | float | Velocity proportional constant | Yes | Changing this parameter causes all velocity PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.vel.i`` | float | Velocity integral constant | Yes | Changing this parameter causes all velocity PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.vel.d`` | float | Velocity derivative constant | Yes | Changing this parameter causes all velocity PID constants for this roboclaw and channel to be written to the target. |
| ``pid.node{idx}.chan{1|2}.vel.qpps`` | int | Max number of encoder pulses per second | Yes | Changing this parameter causes all velocity PID constants for this roboclaw and channel to be written to the target. |

#### Topics
| Action | Topic | Type | Notes |
| :------------- |:-------------| :-----| :-----|
| publish | ``~/claw{idx}/posn_out`` | roboclaw/msg/EncoderStepsStamped | Active only if ``posn_hz`` > 0 |
| publish | ``~/claw{idx}/posn_err`` | roboclaw/msg/EncoderStepsStamped | Active only if ``posn_err_hz`` > 0 |
| publish | ``~/claw{idx}/volts_amps_out`` | roboclaw/msg/MotorVoltsAmpsStamped | Active only if ``volt_amp_hz`` > 0 |
| publish | ``~/claw{idx}/velocity_out`` | roboclaw/msg/EncoderVelocityStamped | Active only if ``velocity_hz`` > 0 |
| publish | ``~/claw{idx}/status`` | roboclaw/msg/StatusStamped | Active only if ``status_hz`` > 0 |
| publish | ``~/claw{idx}/pwm_out`` | roboclaw/msg/MotorPwmStamped | Active only if ``pwm_hz`` > 0 |
| subscribe | ``~/claw{idx}/motor_vel_cmd`` | roboclaw/msg/MotorVelocityStamped | Active only if ``enable.node{idx}.velocity.both`` is true |
| subscribe | ``~/claw{idx}/motor_vel_single_cmd/chan{1|2}`` | roboclaw/msg/MotorVelocitySingleStamped | Active only if ``enable.node{idx}.velocity.chan{1|2}`` is true |
| subscribe | ``~/claw{idx}/motor_pos_cmd`` | roboclaw/msg/MotorPositionStamped | Active only if ``enable.node{idx}.position.both`` is true |
| subscribe | ``~/claw{idx}/motor_pos_single_cmd/chan{1|2}`` | roboclaw/msg/MotorPositionSingleStamped | Active only if ``enable.node{idx}.position.chan{1|2}`` is true |
| subscribe | ``~/claw{idx}/motor_duty_single_cmd/chan{1|2}`` | roboclaw/msg/MotorDutySingleStamped | Active only if ``enable.node{idx}.duty.chan{1|2}`` is true |

#### Services
| Service | Type | Description | Notes |
| :------------- |:-------------| :-----| :-----|
| ``~/roboclaw/get_current_limit | roboclaw/srv/GetCurrentLimit | Read the current limit for a selected Roboclaw and channel. | |
| ``~/roboclaw/get_position_pid | roboclaw/srv/GetPositionPid | Read the position PID constants for a selected Roboclaw and channel. | |
| ``~/roboclaw/get_velocity_pid | roboclaw/srv/GetVelocityPid | Read the velocity PID constants for a selected Roboclaw and channel. | |
| ``~/roboclaw/read_eeprom | roboclaw/srv/ReadEeprom | Reset the configuration of the target Roboclaw to that stored in its onboard EEPROM | This will override values set by parameters and other services until those are called again. |
| ``~/roboclaw/reset_encoder | roboclaw/srv/ResetEncoder | Reset the incremental encoder value of the target Roboclaw and channel to 0 | |
| ``~/roboclaw/reset_motor | roboclaw/srv/ResetMotor | Sends a PWM command of 0 to the target channel of the target Roboclaw; this resets all errors currently set. | |
| ``~/roboclaw/set_current_limit | roboclaw/srv/SetCurrentLimit | Set the current limit for a selected Roboclaw and channel. | Note that this overrides any values set by parameters until the parameter is reset. |
| ``~/roboclaw/set_position_pid | roboclaw/srv/SetPositionPid | Set the position PID constants for a selected Roboclaw and channel. | Note that this overrides any values set by parameters until the parameter is reset. |
| ``~/roboclaw/set_velocity_pid | roboclaw/srv/SetVelocityPid | Set the velocity PID constants for a selected Roboclaw and channel. | Note that this overrides any values set by parameters until the parameter is reset. |
| ``~/roboclaw/write_eeprom | roboclaw/srv/WriteEeprom | Write the current configuration of the target Roboclaw to its onboard EEPROM | |


#### Notes

- {idx} is the index of the target Roboclaw controller
- chan{1|2} selects between channels 1 and 2 of the roboclaw
- When using one Roboclaw controller, configure it in packet serial mode with address 0x80. This will be motor index 0 in RoboclawEncoderSteps and RoboclawMotorVelocity.
- When using more than one Roboclaw controller, configure them in packet serial mode with 0x80 being motor index 0, 0x81 being motor index 1, and so on.

### diffdrive_node

#### This segment has not yet been updated for ROS2. There are no current plans to update it, but PRs doing so will likely be merged. 

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

## TODO

- Tests! PRs that add any sort of usable test coverage will be gratefully merged.
- Cleanup of driver code. There's some historical cruft in the task queue that could be eliminated but I have not had time. 
- Implement additional operational modes. The Roboclaws have a variety of command techniques available and this node only implements some of the more basic ones at this time. 
- Expose the rest of the commands to read/write configuration parameters to the Roboclaw and services and parameters.
- Convert diffdrive_node over to ROS2
- Implement JPL's [Open Source Rover][jpl] drive system. 

[roboclaw]: http://www.basicmicro.com
[ros2]: http://docs.ros.org/en/humble
[jpl]: https://opensourcerover.jpl.nasa.gov
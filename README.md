# Alternate Isaac Kaya Base Driver

This is an alternative to the default [Kaya](https://docs.nvidia.com/isaac/isaac/doc/tutorials/assemble_kaya.html) driver from the [Isaac SDK](https://developer.nvidia.com/isaac-sdk).

The kinematics are calculated following the method discussed in [Mobile Robot Kinematics: Chapter 3](http://www.cs.cmu.edu/~rasc/Download/AMRobots3.pdf) for a 3-wheel holonomic drive.

## General Usage

### Setup

In [WORKSPACE](https://github.com/bluecamel/isaac-kaya/blob/master/WORKSPACE#L34), change the path to your local Isaac SDK.

For example, I have mine set to this:
```
local_repository(
    name = "com_nvidia_isaac",
    path = "/bluecamel/isaac",
)
```

### Move app configuration files

To use this package, you can put your app configuration files in the `packages/kaya/apps` directory.

Everything under this directory operates in the same way as the `apps/kaya` directory in the SDK, but it can't reference configuration files in the SDK `apps` directory.  It can, however, reference configuration files in the SDK `packages` directory.

There probably is a better way to do this, but for now it's easy enough for now.

### Deploy

Deploying is similar to deploying from the SDK.  Instead of running `./engine/build/deploy.sh`, run `./deploy.sh`.

For example, deploy the joystick app:
```
./deploy.sh --remote_user bluecamel -p //packages/kaya/apps:joystick_debug-pkg -d jetpack42 -h 0.0.0.0
```

Replace `bluecamel` with your username on the Kaya.  Replace `0.0.0.0` with the Kaya's IP address.

## BaseDriver component API

`isaac.kaya.BaseDriver` is compatible with [isaac.KayaBaseDriver](https://docs.nvidia.com/isaac/isaac/doc/component_api.html#isaac-kayabasedriver), but doesn't currently respect all parameters.

### Incoming messages
- **command** [HolonomicBaseControls: StateProto]: The holonomic command to be sent to the motors.
  - **speed_x** [*double*]: Linear velocity of the robot in the global reference frame's x axis.
  - **speed_y** [*double*]: Linear velocity of the robot in the global reference frame's y axis.
  - **angular_speed** [*double*]: Angular speed of the robot in the global reference frame.

### Outgoing messages
- **state** [HolonomicBaseDynamics: StateProto]: The state of the holonomic base.
  - **speed_x** [*double*]: Linear velocity of the robot in the global reference frame's x axis.  This is calculated from the wheel velocities reported by the Dynamixel motors.
  - **speed_y** [*double*]: Linear velocity of the robot in the global reference frame's y axis.  This is calculated from the wheel velocities reported by the Dynamixel motors.
  - **angular_speed** [*double*]: Angular speed of the robot in the global reference frame.  This is calculated from the wheel velocities reported by the Dynamixel motors.
  - **acceleration_x** [*double*]: Linear acceleration of the robot  in the global reference frame's x axis.  This is calculated from the current and previous velocities, as reported by the Dynamixel motors.
  - **acceleration_y** [*double*]: Linear acceleration of the robot  in the global reference frame's y axis.  This is calculated from the current and previous velocities, as reported by the Dynamixel motors.

### Sight Channels
These channels are reported to sight if the **report_to_sight** parameter is set to *true*.

![alt text](https://raw.githubusercontent.com/bluecamel/isaac-kaya/master/docs/png/channels.png)

#### command
Clamped input speeds and calculated servo speeds as well as the original command message.

##### message [*HolonomicBaseControls: StateProto*]
The holonomic command to be sent to the motors.
  - **command.message.angular_speed** [*double*]: Angular speed of the robot in the global reference frame.
  - **command.message.speed_x** [*double*]: Linear velocity of the robot in the global reference frame's x axis.
  - **command.message.speed_y** [*double*]: Linear velocity of the robot in the global reference frame's y axis.

##### safe_speed
The speeds, clamped according to *max_safe_speed* and *max_angular_speed* parameters.
  - **command.safe_speed.angle** [*double*]: Angular speed of the robot in the global reference frame.
  - **command.safe_speed.x** [*double*]: Linear velocity of the robot in the global reference frame's x axis.
  - **command.safe_speed.y** [*double*]: Linear velocity of the robot in the global reference frame's y axis.

##### servo
The calculated speeds to be commanded to the servos.
  - **command.servo_back_rad_per_sec** [*double*]: Speed to set the back servo to (in rad/sec).
  - **command.servo_front_left_rad_per_sec** [*double*]: Speed to set the front left servo to (in rad/sec).
  - **command.servo_front_right_rad_per_sec** [*double*]: Speed to set the front right servo to (in rad/sec).

#### current
State reported by the servos.

##### servo speeds
The current speeds reported by the servos.
  - **current.motor_back_rad_per_sec** [*double*]: Current speed reported by the back servo.
  - **current.motor_front_left_rad_per_sec** [*double*]: Current speed reported by the front left servo.
  - **current.motor_front_right_rad_per_sec** [*double*]: Current speed reported by the front right servo.

##### ticks
[Realtime ticks](http://emanual.robotis.com/docs/en/dxl/mx/mx-12w/#realtime-tick) reported by the servos.
  - **current.servo_back_ticks** [*double*]: Current realtime ticks reported by the back servo.
  - **current.servo_front_left_ticks** [*double*]: Current realtime ticks reported by the front left servo.
  - **current.servo_front_right_ticks** [*double*]: Current realtime ticks reported by the front right servo.

**NOTE:** The following parameters are currently ignored: `max_safe_speed`, `max_angular_speed`, `debug_speed`, `debug_servos`.

### Parameters
- **baudrate** [*int*] [default=1000000]: Baud rate of the Dynamixel bus. This is the rate of information transfer.
- **debug_mode** [*bool*] [default=false]: If true, the dynamixel driver will be very chatty will print out details about every command that is being run.  *This currently doesn't behave in the same way as `isaac.KayaBaseDriver`.*
- **dynamixel_protocol_version** [*float*] [default=1.0]: The protocol version to use when communicatin with the Dynamixel motors (e.g. `1.0` or `2.0`).
- **max_angular_speed** [*double*] [default=0.3]: Max turning rate.  **_Currently ignored._**
- **max_safe_speed** [*double*] [default=0.3]: Max safe speed.  **_Currently ignored._**
- **orthogonal_rotation_angle** [*double*] [default=0]: The angle of rotation between the robot frame and global frame.
- **report_to_sight** [*bool*] [default=false]: Report messages and servo info to sight.
- **servo_back** [*int*] [default=2]: Unique identifier for Dynamixel servo at back. Each motor needs to be assigned a unique ID using the software provided by Dynamixel.
- **servo_front_left** [*int*] [default=1]: Unique identifier for Dynamixel servo at front left. Each motor needs to be assigned a unique ID using the software provided by Dynamixel.
- **servo_front_right** [*int*] [default=3]: Unique identifier for Dynamixel servo at front right. Each motor needs to be assigned a unique ID using the software provided by Dynamixel.
- **torque_limit** [*double*] [default=0.5]: Servo maximum torque limit. Caps the amount of torque the servo will apply. 0.0 is no torque, 1.0 is max available torque.
- **wheel_base_length** [*double*] [default=0.125]: Distance of the wheel from robot center of mass. This value is used in kinematic computations.
- **wheel_1_angle** [*double*] [default=$\pi$/3]: Angle in the robot reference frame to the first wheel.
- **wheel_2_angle** [*double*] [default=$\pi$]: Angle in the robot reference frame to the second wheel.
- **wheel_3_angle** [*double*] [default=-$\pi$/3]: Angle in the robot reference frame to the third wheel.
- **wheel_radius** [*double*] [default=0.04]: Wheel radius. This value is used in kinematic computations.
- **usb_port** [*string*] [default="/dev/ttyUSB0"]: USB port where Dynamixel controller is located at. usb_port varies depending on the controller device (e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`).


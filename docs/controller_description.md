# Controller Description

- [Controller Description](#controller-description)
  - [Driver Library Functions](#driver-library-functions)
  - [Keyboard](#keyboard)
  - [InertialUnit](#inertialunit)
  - [Accelerometer](#accelerometer)
  - [GPS](#gps)
  - [Robot](#robot)

A controller is a program that defines the behavior of a robot. Webots controllers can be written in the following programming languages: C, C++, Java, Python, MATLAB, ROS, etc. In this tutorial, we are going to use C as a reference language.

This section describes functions used in the controller of Kubota and McCormick.



## Driver Library Functions



The driver library provides all the usual functionalities available to a human driving his own car.All the functions included in our controller are explained below.For more detailed about functions in driver library, please refer to [Driver LIB](https://www.cyberbotics.com/doc/automobile/driver-library) 


* `wbu_driver_init` Initialise a driver, it should be called at the very beginning of any controller program. 
* `wbu_driver_cleanup` Clean a driver step
* `wbu_driver_step` `step` function should be called in the main loop to run one simulation step.
* `wbu_driver_set_brake_intensity` This function brakes the car by increasing the **dampingConstant** coefficient of the rotational joints of each of the four wheels. The argument should be between 0.0 and 1.0, 0 means that no damping constant is added on the joints (no breaking), 1 means that the parameter **brakeCoefficient** of the [Car](https://www.cyberbotics.com/doc/automobile/proto-nodes) PROTO is applied on the **dampingConstant** of each joint (the value will be linearly interpolated between 0 and **brakeCoefficient** for any arguments between 0 and 1.
* `wbu_driver_set_throttle` This function is used in order to control the car in torque, it sets the state of the throttle.
The argument should be between 0.0 and 1.0, 0 means that 0% of the output torque of the engine is sent to the wheels and 1.0 means that 100% of the output torque of the engine is sent to the wheels.
For more information about how the output torque of the engine is computed see section [Engine models](https://www.cyberbotics.com/doc/automobile/driver-library#engine-models).
* `wbu_driver_set_gear` This function sets the engaged gear.
An argument of `-1` is used in order to engage the reverse gear, an argument of `0` is used in order to disengaged the gearbox.
Any other arguments than `0` and `-1` should be between 1 and the number of coefficients set in the `gearRatio` parameter of the [Car](https://www.cyberbotics.com/doc/automobile/proto-nodes) PROTO.


## Keyboard
The Keyboard is a set of functions available by default for each Robot node to read the keyboard input. It is therefore not a device and the functions do not require any `WbDeviceTag`. For more detailed about functions in driver library, please refer to [Keyboard](https://www.cyberbotics.com/doc/reference/keyboard) 


* `wb_keyboard_enable` Enable keyboard input by calling this function.
* `wb_keyboard_disable` Stop the keyboard reading.
* `wb_keyboard_get_sampling_period` Sampling_period is expressed in milliseconds, and defines how frequently readings are updated, this function will get the sampling_period.
* `wb_keyboard_get_key` Values can be read by calling this function repeatedly untill this function returns -1. 




## InertialUnit


The [InertialUnit](https://www.cyberbotics.com/doc/reference/inertialunit) node simulates an *Inertial Measurement Unit* (IMU).
The InertialUnit computes and returns its *roll*, *pitch* and *yaw* angles with respect to a global coordinate system defined in the WorldInfo node.


* `wb_inertial_unit_enable` Turns on the angle measurements.
* `wb_inertial_unit_get_roll_pitch_yaw` Returns the current *roll*, *pitch* and *yaw* angles of the InertialUnit. The values are returned as an array of 3 components therefore only the indices 0, 1, and 2 are valid for accessing the returned array.
Note that the indices 0, 1 and 2 return the *roll*, *pitch* and *yaw* angles respectively.

> **Note**

>The *roll* angle indicates the unit's rotation angle about its *x*-axis, in the interval [-&pi;,&pi;].
>The *roll* angle is zero when the InertialUnit is horizontal, i.e., when its *y*-axis has the opposite direction of the gravity (WorldInfo defines the `gravity` vector).

>The *pitch* angle indicates the unit's rotation angle about is *z*-axis, in the interval [-&pi;/2,&pi;/2].
>The *pitch* angle is zero when the InertialUnit is horizontal, i.e., when its *y*-axis has the opposite direction of the gravity.
>If the InertialUnit is placed on the Robot with a standard orientation, then the *pitch* angle is negative when the Robot is going down, and positive when the robot is going up.

>The *yaw* angle indicates the unit orientation, in the interval [-&pi;,&pi;], with respect to WorldInfo.`northDirection`.
>The *yaw* angle is zero when the InertialUnit's *x*-axis is aligned with the north direction, it is &pi;/2 when the unit is heading east, and -&pi;/2 when the unit is oriented towards the west.
>The *yaw* angle can be used as a compass.

> [C, C++]: The returned vector is a pointer to internal values managed by the Webots, therefore it is illegal to free this pointer.
> Furthermore, note that the pointed values are only valid until the next call to the `wb_robot_step` or `Robot::step` functions.
> If these values are needed for a longer period they must be copied.




## Accelerometer

The [Accelerometer](https://www.cyberbotics.com/doc/reference/accelerometer) node can be used to model accelerometer devices such as those commonly found in mobile electronics, robots and game input devices.
The Accelerometer node measures acceleration and gravity induced reaction forces over 1, 2 or 3 axes.
It can be used for example to detect fall, the up/down direction, etc.


* `wb_accelerometer_enable` Allows the user to enable the acceleration measurements.
* `wb_accelerometer_get_values` Returns the current values measured by the Accelerometer. These values are returned as a 3D-vector, therefore only the indices 0, 1, and 2 are valid for accessing the vector. Each element of the vector represents the acceleration along the corresponding axis of the Accelerometer node, expressed in meters per second squared [m/s^2].
The first element corresponds to the x-axis, the second element to the y-axis, etc.



> **Note** [C, C++]: The returned vector is a pointer to the internal values managed by the Accelerometer node, therefore it is illegal to free this pointer.
> Furthermore, note that the pointed values are only valid until the next call to the `wb_robot_step` or `Robot::step` functions.
> If these values are needed for a longer period they must be copied.



## GPS


The [GPS](https://www.cyberbotics.com/doc/reference/gps) node is used to model a Global Positioning Sensor (GPS) which can obtain information about its absolute position from the controller program.


* `wb_gps_enable` Allows the user to enable GPS measurements.
* `wb_gps_get_values` Returns the current GPS measurement.
The values are returned as a 3D-vector, therefore only the indices 0, 1, and 2 are valid for accessing the vector.
The returned vector indicates the absolute position of the GPS device. This position can either be expressed in the cartesian coordinate system of Webots or using latitude-longitude-altitude, depending on the value of the `gpsCoordinateSystem` field of the WorldInfo node.



> **Note** [C, C++]: The returned vector is a pointer to the internal values managed by the GPS node, therefore it is illegal to free this pointer.
> Furthermore, note that the pointed values are only valid until the next call to the `wb_robot_step` or `Robot::step` functions.
> If these values are needed for a longer period they must be copied.



## Robot


The [Robot](https://www.cyberbotics.com/doc/reference/robot) node can be used as basis for building a robot, e.g., an articulated robot, a humanoid robot, a wheeled robot.

> **Note**: Logically, if the Robot node has one or more Solid (or derived) ancestor nodes, then the physical properties of the ancestor nodes will affect the Robot node's physical behavior.


* `wb_robot_get_device` Returns a unique identifier for a device corresponding to a specified `name`.
For example, if a robot contains a DistanceSensor node whose `name` field is "ds1", the function will return the unique identifier of that device.
This `WbDeviceTag` identifier will be used subsequently for enabling, sending commands to, or reading data from this device.
If the specified device is not found, the function returns 0.

* `wb_robot_get_basic_time_step` Returns the value of the basicTimeStep field of the WorldInfo node
# CTRE Swerve

7525's implementation of the CTRE Swerve API with Akit support. Will likley be used in the 2025 season for 7525.

## Code Structure
The Pioneers lib folder has custom subsystem abstract classes that we use for our project structure. We use timed robot with state machines and have IO layers for all our subsystems.

## Controller Bindings
An XboxController is required

**Right Joystick X** - Robot rotation, left corresponds to counter clockwise rotation and vice versa
**Left Joystick Y** - Robot translation in the y direction (forwards and backwards) relative to the field or robot depending on the mode. Forward on the joystick correspodns to forward.
**Left Joystick X** - Robot translation in the x direction (left and right) relative to the field or robot depending on the mode. Left on the joystick corresponds to a left translation.
**Back Button** - Toggles between field relative and robot relative driving. Default is field relative.
**Start Button** - Manually zeroes the gyroscope. For use when vision is not available.
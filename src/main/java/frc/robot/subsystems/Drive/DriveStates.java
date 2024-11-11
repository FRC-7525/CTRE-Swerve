package frc.robot.subsystems.Drive;

import frc.robot.pioneersLib.subsystem.SubsystemStates;

import static frc.robot.Constants.Controllers.*;

/**
 * An enumeration representing different drive states for a robot's drive subsystem.
 */
public enum DriveStates implements SubsystemStates {
    /**
     * The robot's drive is in field-relative mode.
     */
    FIELD_RELATIVE("Field Relative", () -> {Drive.getInstance().driveFieldRelative(DRIVER_CONTROLLER.getRightX(), DRIVER_CONTROLLER.getRightY(), DRIVER_CONTROLLER.getLeftX());}),

    /**
     * The robot's drive is in robot-relative mode.
     */
    ROBOT_RELATIVE("Robot Relative", () -> {Drive.getInstance().driveRobotRelative(DRIVER_CONTROLLER.getRightX(), DRIVER_CONTROLLER.getRightY(), DRIVER_CONTROLLER.getLeftX());}),

    /**
     * The robot's drive is in locking wheels mode starting from field relative.
     */
    LOCKING_WHEELS_FIELD("Locking Wheels", () -> {Drive.getInstance().lockWheels();}),

    /**
     * The robot's drive is in locking wheels mode starting from robot relative.
     */
    LOCKING_WHEELS_ROBOT("Locking Wheels Robot", () -> {Drive.getInstance().lockWheels();}),;

    private String stateString;
    private Runnable driveControl;

    /**
     * Constructs a DriveStates enum value with the specified state string and field-relative flag.
     * 
     * @param stateString    the string representation of the drive state
     * @param Runnable  runnable that gets called to drive the robot
     */
    DriveStates(String stateString, Runnable driveControl) {
        this.stateString = stateString;
        this.driveControl = driveControl;
    } 

    /**
     * Returns the string representation of the drive state.
     * 
     * @return the string representation of the drive state
     */
    @Override
    public String getStateString() {
        return stateString;
    }

    /**
     * Drives the robot in the current state.
     */
    public void driveRobot() {
        driveControl.run();
    }
}

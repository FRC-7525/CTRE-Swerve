package frc.robot.subsystems.Drive;

import frc.robot.pioneersLib.subsystem.SubsystemStates;

/**
 * An enumeration representing different drive states for a robot's drive subsystem.
 */
public enum DriveStates implements SubsystemStates {
    /**
     * The robot's drive is in field-relative mode.
     */
    FIELD_RELATIVE("Field Relative"),

    /**
     * The robot's drive is in robot-relative mode.
     */
    ROBOT_RELATIVE("Robot Relative"),

    /**
     * The robot's drive is in locking wheels mode starting from field relative.
     */
    LOCKING_WHEELS_FIELD("Locking Wheels"),

    /**
     * The robot's drive is in locking wheels mode starting from robot relative.
     */
    LOCKING_WHEELS_ROBOT("Locking Wheels Robot");

    private String stateString;
    private boolean fieldRelative;

    /**
     * Constructs a DriveStates enum value with the specified state string and field-relative flag.
     * 
     * @param stateString    the string representation of the drive state
     * @param fieldRelative  true if the drive state is field-relative, false otherwise
     */
    DriveStates(String stateString) {
        this.stateString = stateString;
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
     * Returns true if the drive state is field-relative, false otherwise.
     * 
     * @return true if the drive state is field-relative, false otherwise
     */
    public boolean getFieldRelative() {
        return fieldRelative;
    }
}

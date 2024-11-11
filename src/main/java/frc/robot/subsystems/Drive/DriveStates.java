package frc.robot.subsystems.Drive;

import frc.robot.pioneersLib.subsystem.SubsystemStates;

/**
 * An enumeration representing different drive states for a robot's drive subsystem.
 */
public enum DriveStates implements SubsystemStates {
    /**
     * The robot's drive is in field-relative mode.
     */
    FIELD_RELATIVE("Field Relative", true),

    /**
     * The robot's drive is in robot-relative mode.
     */
    ROBOT_RELATIVE("Robot Relative", false),

    /**
     * The robot's drive is in locking wheels mode.
     */
    LOCKING_WHEELS("Locking Wheels", false);

    private String stateString;
    private boolean fieldRelative;

    /**
     * Constructs a DriveStates enum value with the specified state string and field-relative flag.
     * 
     * @param stateString    the string representation of the drive state
     * @param fieldRelative  true if the drive state is field-relative, false otherwise
     */
    DriveStates(String stateString, boolean fieldRelative) {
        this.stateString = stateString;
        this.fieldRelative = fieldRelative;
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

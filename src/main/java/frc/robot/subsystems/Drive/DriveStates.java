package frc.robot.subsystems.Drive;

import frc.robot.pioneersLib.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
    FIELD_RELATIVE("Field Relative", true),
    ROBOT_RELATIVE("Robot Relative", false),
    LOCKING_WHEELS("Locking Wheels", false);

    DriveStates(String stateString, boolean fieldRelative) {
        this.stateString = stateString;
        this.fieldRelative = fieldRelative;
    } 

    private String stateString;
    private boolean fieldRelative;

    @Override
    public String getStateString() {
        return stateString;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }
}

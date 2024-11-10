package frc.robot.subsystems.Manager;

import frc.robot.pioneersLib.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
    IDLE("Idle");
    
    ManagerStates(String stateString) {
        this.stateString = stateString;
    }

    private String stateString;

    @Override
    public String getStateString() {
        return stateString;
    }
}

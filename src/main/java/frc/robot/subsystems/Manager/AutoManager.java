package frc.robot.subsystems.Manager;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoManager {
    
    private static AutoManager instance;
    private SendableChooser<Command> autoChooser;

    private AutoManager() {
        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("NO AUTO SELECTED", new PrintCommand("No Auto Selected, nothing ran"));

        // AUTOS GO HERE!
        autoChooser.addOption("PID Tuning Auto", new PathPlannerAuto("PID Tuning Auto"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Gets the instance of the AutoManager
     * @return
     */
    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }

    /**
     * Gets the selected auto, should be scheduled in auto init
     */
    public Command getSelectedAuto() {
        Command autoCommand = autoChooser.getSelected();
        return autoCommand;
    }   
}

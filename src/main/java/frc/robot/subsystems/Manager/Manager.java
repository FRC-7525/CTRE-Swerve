package frc.robot.subsystems.Manager;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Controllers.TEST_CONTROLLER;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Vision.Vision;

public class Manager extends Subsystem<ManagerStates> {

    private static Manager instance;

    private final Drive drive = Drive.getInstance();
    private final Vision vision = Vision.getInstance();
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    // Change to change the subsystem that gets tested (has runnable sysID tests) saftey ish
    private final Subsystem<?> sysIdSubsystem = drive;


    private Manager() {
        super("Manager", ManagerStates.IDLE);

        // State transitions go here

        // SYS ID Tests (see constants to change test type for drive)
        if (ROBOT_MODE == RobotMode.TESTING) {
            addRunnableTrigger(() -> {commandScheduler.schedule(sysIdSubsystem.sysIdDynamic(Direction.kForward));}, () -> TEST_CONTROLLER.getAButtonPressed());
            addRunnableTrigger(() -> {commandScheduler.schedule(sysIdSubsystem.sysIdDynamic(Direction.kReverse));}, () -> TEST_CONTROLLER.getBButtonPressed());
            addRunnableTrigger(() -> {commandScheduler.schedule(sysIdSubsystem.sysIdQuasistatic(Direction.kForward));}, () -> TEST_CONTROLLER.getXButtonPressed());
            addRunnableTrigger(() -> {commandScheduler.schedule(sysIdSubsystem.sysIdQuasistatic(Direction.kReverse));}, () -> TEST_CONTROLLER.getYButtonPressed());
        }
    }

    public static Manager getInstance() {
        if (instance == null) {
            instance = new Manager();
        }
        return instance;
    }

    @Override
    public void runState() {

        Logger.recordOutput("Manager/State", getState().getStateString());
        Logger.recordOutput("Manager/State Time", getStateTime());

        drive.periodic();
        vision.periodic();

        // Other subsystem periodics go here
    }
}

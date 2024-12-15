// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Manager.AutoManager;
import frc.robot.pioneersLib.misc.CommandsUtil;
import frc.robot.subsystems.Manager.Manager;

public class Robot extends LoggedRobot {

    private final Manager manager = Manager.getInstance();
    private final AutoManager autoManager = AutoManager.getInstance();

    @Override
    public void robotInit() {

        switch (Constants.ROBOT_MODE) {
            case REAL:
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(new WPILOGWriter());
                break;
        
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case TESTING:
                Logger.addDataReceiver(new NT4Publisher());
                break;
        }

		Logger.start();
        CommandsUtil.logCommands();
		DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        manager.periodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(autoManager.getSelectedAuto());
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class DriveIOSim extends DriveIOReal {

    private double lastSimTime;
    private Notifier simNotifier;

    public DriveIOSim() {
        super();
        startSimThread();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            getDrive().updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(Constants.Drive.SIM_UPDATE_TIME);
    }
}

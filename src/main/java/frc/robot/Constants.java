package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive.Drive.SysIdMode;

public final class Constants {

    public enum RobotMode {
        REAL,
        TESTING,
        SIM
    }

    public static final RobotMode ROBOT_MODE = RobotMode.SIM;

    public static class Controllers {
        public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
        public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);
        public static final XboxController TEST_CONTROLLER = new XboxController(3);
        
        // NOTE: Set to 0.1 on trash controllers
        public static final double DEADBAND = 0.01;
    }

    public static class Drive {
        public static final double SIM_UPDATE_TIME = 0.004;

        public static final AngularVelocity ANGULAR_VELOCITY_LIMIT = AngularVelocity.ofBaseUnits(180,  DegreesPerSecond);

        // Change to change the sysID test that gets run for drive
        public static final SysIdMode SYS_ID_MODE = SysIdMode.STEER;
        public static final String SUBSYSTEM_NAME = "Drive";

        // For zeroing on robot init
        public static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
        public static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);           
    }
}

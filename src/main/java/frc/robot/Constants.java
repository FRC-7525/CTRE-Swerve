package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive.Drive.SysIdMode;

public final class Constants {

    public enum RobotState {
        REAL,
        TESTING,
        SIM
    }

    public static final RobotState ROBOT_STATE = RobotState.REAL;

    public static class Controllers {
        public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
        public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);
        public static final XboxController TEST_CONTROLLER = new XboxController(3);

        public static final double DEADBAND = 0.1;
    }

    public static class Drive {
        public static final double SIM_UPDATE_TIME = 0.004;
        // Change to change the sysID test that gets run for drive
        public static final SysIdMode SYS_ID_MODE = SysIdMode.STEER;

        // For zeroing on robot init
        public static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
        public static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);           
    }
}

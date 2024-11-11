package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class DriveOutputManager {
    static final String sussystemname = "Drive";
    static Pose2d lastPose = new Pose2d();
    static double lastTime = 0;

    public static void processSwerveDriveState(SwerveDriveState swerveDriveState) {
        Pose2d currentPose2d = swerveDriveState.Pose;
        double currentTime = Utils.getSystemTimeSeconds();
        Translation2d distanceDiff = currentPose2d.minus(lastPose).getTranslation();
        lastPose = currentPose2d;
        lastTime = currentTime;

        Logger.recordOutput("Drive/RealOutput/robotPose", currentPose2d);
        Logger.recordOutput("Drive/RealOutput/currentTIme", currentTime);
        Logger.recordOutput("Drive/RealOutput/distanceDiff", distanceDiff);
        Logger.recordOutput("Drive/RealOutput/chassisSpeed", swerveDriveState.Speeds);
        Logger.recordOutput("Drive/RealOutput/velocity", Units.metersToFeet(Math.hypot(swerveDriveState.Speeds.vxMetersPerSecond, swerveDriveState.Speeds.vyMetersPerSecond)));
        Logger.recordOutput("Drive/RealOutput/swerveModuleStates", swerveDriveState.ModuleStates);
        Logger.recordOutput("Drive/RealOutput/swerveModulePosition", swerveDriveState.ModulePositions);

    }

    
}

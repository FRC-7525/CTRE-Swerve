package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveOutputManager {
    static Pose2d lastPose = new Pose2d();
    static double lastTime = 0;

    public static void processSwerveDriveState(SwerveDriveState swerveDriveState) {
        Pose2d currentPose2d = swerveDriveState.Pose;
        double currentTime = Utils.getSystemTimeSeconds();
        double timeDiff = currentTime - lastTime;
        Translation2d distanceDiff = currentPose2d.minus(lastPose).getTranslation();
        lastPose = currentPose2d;
        lastTime = currentTime;
        

        Logger.recordOutput("Drive/RealOutput/robotPose", currentPose2d);
        Logger.recordOutput("Drive/RealOutput/currentTIme", currentTime);
        Logger.recordOutput("Drive/RealOutput/distanceDiff", distanceDiff);
        Logger.recordOutput("Drive/RealOutput/velocity", distanceDiff.div(timeDiff));
        Logger.recordOutput("Drive/RealOutput/swerveModuleStates", swerveDriveState.ModuleStates);
        Logger.recordOutput("Drive/RealOutput/swerveModulePosition", swerveDriveState.ModulePositions);

    }

    
}

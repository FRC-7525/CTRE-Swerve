package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface DriveIO {

    public class DriveIOInputs {
        ChassisSpeeds speeds = new ChassisSpeeds();
        SwerveModuleState[] setPoints = new SwerveModuleState[4];
        Pose2d pose = new Pose2d();
        Rotation3d fullRobotRotation = new Rotation3d();
        double odometryFrequency = 0;
        double timestamp = 0;
        double failedDataAquisitions = 0;
        double robotAngleDeg = 0;
    }

    public default void updateInputs(DriveIOInputs inputs) {}

    public default SwerveDrivetrain getDrive() {
        return null;
    }

    public default void zeroGyro() {}

    public default void setControl(SwerveRequest request) {}    
    
    public default void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {}
}

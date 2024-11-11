package frc.robot.subsystems.Drive;

import static frc.robot.subsystems.Drive.TunerConstants.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class DriveIOReal implements DriveIO {

    public DriveIOInputs inputs = new DriveIOInputs();
    private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.speeds = drivetrain.getState().Speeds;
        inputs.setPoints = drivetrain.getState().ModuleTargets;
        inputs.odometryFrequency = drivetrain.getOdometryFrequency();
        inputs.timestamp = drivetrain.getState().Timestamp;
        inputs.failedDataAquisitions = drivetrain.getState().FailedDaqs;
        inputs.robotAngleDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        inputs.pose = drivetrain.getState().Pose;
        inputs.fullRobotRotation = drivetrain.getRotation3d();
        inputs.gyroAngleDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }

    @Override
    public SwerveDrivetrain getDrive() {
        return drivetrain;
    }   

    @Override
    public void zeroGyro() {
        drivetrain.resetRotation(new Rotation2d());
    }

    @Override
    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {
        drivetrain.addVisionMeasurement(pose, timestamp, standardDeviaton);
    }
}

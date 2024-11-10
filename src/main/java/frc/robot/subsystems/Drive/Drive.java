package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.pioneersLib.subsystem.Subsystem;

import static frc.robot.subsystems.Drive.TunerConstants.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Controllers.*;

public class Drive extends Subsystem<DriveStates> {
    private static Drive instance;

    private SwerveDrivetrain drivetrain;
    // I don't like this also (I did this one tho)
    private Telemetry telemetryLogger;
    private boolean zeroedWithRespectToAlliance = false;

    private double lastSimTime;
    private Notifier simNotifier;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private Drive() {
        super("Drive", DriveStates.FIELD_RELATIVE);
        drivetrain = new SwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
        // I don't like this either
        telemetryLogger = new Telemetry(kSpeedAt12Volts.in(MetersPerSecond));
        drivetrain.registerTelemetry(telemetryLogger::telemeterize);

        // TODO: I really don't like this
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // Zero Gyro
        addRunnableTrigger(() -> {
            drivetrain.resetRotation(new Rotation2d(0));
        }, () -> DRIVER_CONTROLLER.getStartButtonPressed());

        // Field to relative and whatnot
        addTrigger(DriveStates.FIELD_RELATIVE, DriveStates.ROBOT_RELATIVE,
                () -> DRIVER_CONTROLLER.getBackButtonPressed());
        addTrigger(DriveStates.ROBOT_RELATIVE, DriveStates.FIELD_RELATIVE,
                () -> DRIVER_CONTROLLER.getBackButtonPressed());

        // Kinda weird state issue I've never thought abt this so when you go into
        // lockking wheels you can't go back to robot relative
        addTrigger(DriveStates.FIELD_RELATIVE, DriveStates.LOCKING_WHEELS,
                () -> DRIVER_CONTROLLER.getLeftBumperButtonPressed());
        addTrigger(DriveStates.ROBOT_RELATIVE, DriveStates.LOCKING_WHEELS,
                () -> DRIVER_CONTROLLER.getLeftBumperButtonPressed());
        addTrigger(DriveStates.LOCKING_WHEELS, DriveStates.FIELD_RELATIVE,
                () -> DRIVER_CONTROLLER.getLeftBumperButtonPressed());
    }

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private SwerveDrivetrain getDrive() {
        return drivetrain;
    }

    @Override
    public void runState() {
        // I mean.. I guess... but wtv
        if (!zeroedWithRespectToAlliance || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                getDrive().setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? redAlliancePerspectiveRotation
                                : blueAlliancePerspectiveRotation);
                zeroedWithRespectToAlliance = true;
            });
        }

        if (getState() == DriveStates.FIELD_RELATIVE) {
            driveFieldRelative(DRIVER_CONTROLLER.getLeftX(), DRIVER_CONTROLLER.getLeftY(),
                    DRIVER_CONTROLLER.getRightX());
        } else if (getState() == DriveStates.ROBOT_RELATIVE) {
            driveRobotRelative(DRIVER_CONTROLLER.getLeftX(), DRIVER_CONTROLLER.getLeftY(),
                    DRIVER_CONTROLLER.getRightY());
        } else {
            lockWheels();
        }

        // Ughhhh someone install a logger (akit)
    }

    // STAND BACK I'M GONA CRASH OUT!
    public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withDeadband(DEADBAND)
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(angularVelocity)
                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
    }

    public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
        drivetrain.setControl(
                new SwerveRequest.RobotCentric()
                        .withDeadband(DEADBAND)
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(angularVelocity)
                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
    }

    public void lockWheels() {
        drivetrain.setControl(
                new SwerveRequest.SwerveDriveBrake()
                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
    }

    public void addVisionMeasument(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {
        drivetrain.addVisionMeasurement(pose, timestamp, standardDeviaton);
    }

    // TODO: I don't like this
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

    // SYSId Trash (no hate ofc)
    public enum SysIdMode {
        TRANSLATION,
        STEER,
        ROTATION
    }

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentrvesicFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        getDrive().setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    Drive.getInstance()));

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> getDrive().setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    Drive.getInstance()));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> getDrive().setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    Drive.getInstance()));

    @Override
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        switch (Constants.Drive.SYS_ID_MODE) {
            case ROTATION:
                return sysIdRoutineRotation.quasistatic(direction);
            case TRANSLATION:
                return sysIdRoutineTranslation.quasistatic(direction);
            case STEER:
                return sysIdRoutineSteer.quasistatic(direction);
            default:
                return new PrintCommand("Invalid SysId mode (Quasistatic)");
        }
    }

    @Override
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        switch (Constants.Drive.SYS_ID_MODE) {
            case ROTATION:
                return sysIdRoutineRotation.dynamic(direction);
            case TRANSLATION:
                return sysIdRoutineTranslation.dynamic(direction);
            case STEER:
                return sysIdRoutineSteer.dynamic(direction);
            default:
                return new PrintCommand("Invalid SysId mode (Dynamic)");
        }
    }

}

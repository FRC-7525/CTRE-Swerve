package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.pioneersLib.subsystem.Subsystem;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Controllers.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Drive.*;

public class Drive extends Subsystem<DriveStates> {
    private static Drive instance;

    // I don't like this also (I did this one tho)
    private DriveIO driveIO;
    private DriveIOInputsAutoLogged inputs;
    private boolean zeroedWithRespectToAlliance = false;
    private Pose2d lastPose = new Pose2d();
    // TODO: Pose jumping detection (last time - current time * velocity > translation * 2)
    @SuppressWarnings("unused")
    private double lastTime = 0;

    /**
     * Constructs a new Drive subsystem with the given DriveIO.
     * 
     * @param driveIO The DriveIO object used for controlling the drive system.
     */
    private Drive(DriveIO driveIO) {
        super("Drive", DriveStates.FIELD_RELATIVE);
        this.driveIO = driveIO;
        // I don't like this either
        // TODO: I really don't like this
        if (Utils.isSimulation()) {
        }

        // Zero Gyro
        addRunnableTrigger(() -> {
            driveIO.zeroGyro();
        }, () -> DRIVER_CONTROLLER.getStartButtonPressed());

        // Field to relative and whatnot
        addTrigger(DriveStates.FIELD_RELATIVE, DriveStates.ROBOT_RELATIVE,
                () -> DRIVER_CONTROLLER.getBackButtonPressed());
        addTrigger(DriveStates.ROBOT_RELATIVE, DriveStates.FIELD_RELATIVE,
                () -> DRIVER_CONTROLLER.getBackButtonPressed());

        // TODO: Kinda weird state issue I've never thought abt this so when you go into
        // lockking wheels you can't go back to robot relative
        addTrigger(DriveStates.FIELD_RELATIVE, DriveStates.LOCKING_WHEELS,
                () -> DRIVER_CONTROLLER.getLeftBumperPressed());
        addTrigger(DriveStates.ROBOT_RELATIVE, DriveStates.LOCKING_WHEELS,
                () -> DRIVER_CONTROLLER.getLeftBumperPressed());
        addTrigger(DriveStates.LOCKING_WHEELS, DriveStates.FIELD_RELATIVE,
                () -> DRIVER_CONTROLLER.getLeftBumperPressed());
    }

    /**
     * Returns the singleton instance of the Drive subsystem.
     * 
     * @return The Drive instance.
     */
    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive(
                    switch (ROBOT_STATE) {
                        case REAL -> new DriveIOReal();
                        case SIM -> new DriveIOSim();
                        case TESTING -> new DriveIOReal();
                    });
        }
        return instance;
    }

    @Override
    public void runState() {
        driveIO.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        // Zero on init/when first disabled
        if (!zeroedWithRespectToAlliance || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                driveIO.getDrive().setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? Constants.Drive.redAlliancePerspectiveRotation
                                : Constants.Drive.blueAlliancePerspectiveRotation);
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

        logOutputs(driveIO.getDrive().getState());
        Logger.recordOutput("Drive/State", getState().getStateString());
    }

    /**
     * Logs the outputs of the drive system.
     * 
     * @param state The current state of the SwerveDrive.
     */
    public void logOutputs(SwerveDriveState state) {
        Logger.recordOutput(SUBSYSTEM_NAME + "/Robot Pose", state.Pose);
        Logger.recordOutput(SUBSYSTEM_NAME + "/Current Time", Utils.getSystemTimeSeconds());
        Logger.recordOutput(SUBSYSTEM_NAME + "/Chassis Speeds", state.Speeds);
        Logger.recordOutput(SUBSYSTEM_NAME + "/velocity",
                Units.metersToFeet(Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)));
        Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModuleStates", state.ModuleStates);
        Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModulePosition", state.ModulePositions);
        Logger.recordOutput(SUBSYSTEM_NAME + "/Translation Difference",
                state.Pose.getTranslation().minus(lastPose.getTranslation()));

        lastPose = state.Pose;
        lastTime = Utils.getSystemTimeSeconds();
    }

    /**
     * Drives the robot in field-relative mode.
     * 
     * @param xVelocity       The desired x-axis velocity.
     * @param yVelocity       The desired y-axis velocity.
     * @param angularVelocity The desired angular velocity.
     */
    public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
        driveIO.setControl(
                new SwerveRequest.FieldCentric()
                        .withDeadband(DEADBAND)
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(angularVelocity)
                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
    }

    /**
     * Drives the robot in robot-relative mode.
     * 
     * @param xVelocity       The desired x-axis velocity.
     * @param yVelocity       The desired y-axis velocity.
     * @param angularVelocity The desired angular velocity.
     */
    public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
        driveIO.setControl(
                new SwerveRequest.RobotCentric()
                        .withDeadband(DEADBAND)
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(angularVelocity)
                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
    }

    /**
     * Locks the wheels of the robot.
     */
    public void lockWheels() {
        driveIO.setControl(
                new SwerveRequest.SwerveDriveBrake()
                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
    }

    public void addVisionMeasument(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviaton) {
        driveIO.addVisionMeasurement(pose, timestamp, standardDeviaton);
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
                        driveIO.getDrive().setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
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
                    output -> driveIO.getDrive().setControl(m_translationCharacterization.withVolts(output)),
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
                    volts -> driveIO.getDrive().setControl(m_steerCharacterization.withVolts(volts)),
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

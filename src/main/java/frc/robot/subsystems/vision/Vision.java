package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.pioneersLib.misc.VisionUtil;
import frc.robot.pioneersLib.subsystem.Subsystem;
import frc.robot.subsystems.Drive.Drive;

import static frc.robot.Constants.Vision.*;

public class Vision extends Subsystem<VisionStates> {

    private VisionIO io;
    private Drive drive;

    public Vision(VisionIO io, Drive drive) {
        super("Vision", VisionStates.ON);

        this.io = io;
        this.drive = drive;
    }

    @Override
    public void runState() {
        if (getState().getVisionEnabled()) {
            io.setStrategy(getState().getStrategy());
            io.updateRobotPose(drive.getPose());
    
            Optional<EstimatedRobotPose> frontPose = io.getFrontPoseEstimation();
            if (io.getFrontPoseEstimation().isPresent()) {
                drive.addVisionMeasurment(frontPose.get().estimatedPose.toPose2d(), frontPose.get().timestampSeconds,
                        VisionUtil.getEstimationStdDevs(frontPose.get(), FRONT_RESOLUTION));
            }
    
            Optional<EstimatedRobotPose> sidePose = io.getSidePoseEstimation();
            if (sidePose.isPresent()) {
                drive.addVisionMeasurment(
                        sidePose.get().estimatedPose.toPose2d(),
                        sidePose.get().timestampSeconds,
                        VisionUtil.getEstimationStdDevs(sidePose.get(), SIDE_RESOLUTION));
            }
        }
    }
}

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelightutil.LimelightHelpers;
import frc.robot.Constants;

public class LimelightVision extends SubsystemBase {
    private String name;

    private VisionPoseMeasurement currentPoseEstimate = new VisionPoseMeasurement();
    private AprilTagTarget currentTarget = new AprilTagTarget();
    
    private boolean isLocalizationPipeline;//Whether we are localizing or tracking a target

    public LimelightVision(String name, boolean isLocalizationPipeline) {
        this.name = name;
        this.isLocalizationPipeline = isLocalizationPipeline;
    }

    @Override
    public void periodic() {
        if(isLocalizationPipeline) {
            currentPoseEstimate.pose = LimelightHelpers.getBotPose2d(name);

            // Subtract the pipeline latency from the starting time to get measurement time
            currentPoseEstimate.measurementTime = Timer.getFPGATimestamp() -
                    (LimelightHelpers.getLatency_Pipeline(name) / 1000.0) -
                    (LimelightHelpers.getLatency_Capture(name) / 1000.0);

            Pose2d currentPose = currentPoseEstimate.pose;

            SmartDashboard.putNumber("Vision Pose X", currentPose.getX());
            SmartDashboard.putNumber("Viision Pose Y", currentPose.getY());
            SmartDashboard.putNumber("Vision Pose Rot", currentPose.getRotation().getDegrees());
        }

        else {
            currentTarget.xOffset = LimelightHelpers.getTX(name);
            currentTarget.yOffset = LimelightHelpers.getTY(name);
            currentTarget.area = LimelightHelpers.getTA(name);

            SmartDashboard.putNumber("Target X", currentTarget.xOffset);//TODO update dashboard
            SmartDashboard.putNumber("Target Y", currentTarget.yOffset);
            SmartDashboard.putNumber("Target Area", currentTarget.area);
        }
    }

    public void switchToTargetPipeline() {
        if(isLocalizationPipeline) {
            LimelightHelpers.setPipelineIndex(name, Constants.VisionConstants.targetPipelineId);

            isLocalizationPipeline = false;
        }
    }

    public void switchToLocalizationPipeline() {
        if(!isLocalizationPipeline) {
            LimelightHelpers.setPipelineIndex(name, Constants.VisionConstants.localizationPipelineId);

            isLocalizationPipeline = true;
        }
    }

    public double getMeasurementTime() {
        return 0;
    }

    public Optional<VisionPoseMeasurement> getPoseEstimate() {
        if(isLocalizationPipeline)
            return Optional.of(currentPoseEstimate);

        else
            return Optional.empty();
    }

    public AprilTagTarget getAprilTagTarget() {
        if (!isLocalizationPipeline)
            return currentTarget;

        else
            throw new IllegalStateException();
    }

    public static final class AprilTagTarget {
        public double xOffset;
        public double yOffset;
        public double area;
    }

    public static final class VisionPoseMeasurement {
        public Pose2d pose;
        public double measurementTime;
    }
}   
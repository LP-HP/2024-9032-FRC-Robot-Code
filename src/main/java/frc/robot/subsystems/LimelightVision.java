package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelightutil.LimelightHelpers;

public class LimelightVision extends SubsystemBase {
    private String name;

    private Pose2d currentPoseEstimate;
    private AprilTagTarget currentTarget = new AprilTagTarget();
    
    private boolean isLocalizationPipeline;//Whether we are localizing or tracking a target

    public LimelightVision(String name, boolean isLocalizationPipeline) {
        this.name = name;
        this.isLocalizationPipeline = isLocalizationPipeline;
    }

    @Override
    public void periodic() {
        if(isLocalizationPipeline) {
            currentPoseEstimate = LimelightHelpers.getBotPose2d(name);
        }

        else {
            currentTarget.xOffset = LimelightHelpers.getTX(name);
            currentTarget.yOffset = LimelightHelpers.getTY(name);
            currentTarget.area = LimelightHelpers.getTA(name);
        }
    }

    public void switchToTargetPipeline() {
        LimelightHelpers.setPipelineIndex(name, 1);//TODO make sure this aligns with the limelight config

        isLocalizationPipeline = false;
    }

    public void switchToLocalizationPipeline() {
        LimelightHelpers.setPipelineIndex(name, 0);

        isLocalizationPipeline = true;
    }

    public double getMeasurementTime() {
        return 
        Timer.getFPGATimestamp() - 
        (LimelightHelpers.getLatency_Pipeline(name) / 1000.0) - 
        (LimelightHelpers.getLatency_Capture(name) / 1000.0);
    }

    public Pose2d getPoseEstimate() {
        if(isLocalizationPipeline)
            return currentPoseEstimate;

        else
            throw new IllegalStateException();
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
}   
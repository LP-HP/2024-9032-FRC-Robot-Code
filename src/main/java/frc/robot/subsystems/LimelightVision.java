package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelightutil.LimelightHelpers;
import frc.robot.Constants;

public class LimelightVision extends SubsystemBase {
    private final String name = Constants.VisionConstants.limelightName;

    private VisionPoseMeasurement lastPoseEstimate = new VisionPoseMeasurement();
    private AprilTagTarget currentTarget = new AprilTagTarget();
    
    private boolean isLocalizationPipeline;//Whether we are localizing or tracking a target

    private final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    public LimelightVision(boolean isLocalizationPipeline) {
        this.isLocalizationPipeline = isLocalizationPipeline;

        limelightTab.addCamera("Limelight View", name, "camera_server://limelight" + name)//TODO does this work
            .withPosition(1, 1).withSize(6, 6);

        limelightTab.add(currentTarget).withPosition(1, 8).withSize(2, 4);
        limelightTab.add(lastPoseEstimate).withPosition(6, 8).withSize(2, 4);
    }

    @Override
    public void periodic() {
        if(isLocalizationPipeline) {
            Pose2d currentPose = LimelightHelpers.getBotPose2d_wpiBlue(name);//TODO what coordinates? should be wpilib blue

            //If we get the same measurement as the last loop then invalidate
            if(currentPose.equals(lastPoseEstimate.pose))
                lastPoseEstimate.isValid = false;

            else {
                lastPoseEstimate.pose = currentPose;
                lastPoseEstimate.isValid = true;

                // Subtract the pipeline latency from the starting time to get measurement time
                lastPoseEstimate.measurementTime = Timer.getFPGATimestamp() -
                        (LimelightHelpers.getLatency_Pipeline(name) / 1000.0) -
                        (LimelightHelpers.getLatency_Capture(name) / 1000.0);
            }
        }

        else {
            currentTarget.xOffset = LimelightHelpers.getTX(name);
            currentTarget.yOffset = LimelightHelpers.getTY(name);
            currentTarget.area = LimelightHelpers.getTA(name);
            currentTarget.isValid = LimelightHelpers.getTV(name);
            currentTarget.id = LimelightHelpers.getFiducialID(name);
        }

        SmartDashboard.putBoolean("Is localization pipeline", isLocalizationPipeline);
    }

    public void switchToTargetPipeline() {
        if(isLocalizationPipeline) {
            LimelightHelpers.setPipelineIndex(name, Constants.VisionConstants.targetPipelineID);

            isLocalizationPipeline = false;
        }
    }

    public void switchToLocalizationPipeline() {
        if(!isLocalizationPipeline) {
            LimelightHelpers.setPipelineIndex(name, Constants.VisionConstants.localizationPipelineID);

            isLocalizationPipeline = true;
        }
    }

    public Optional<VisionPoseMeasurement> getPoseEstimate() {
        //Do not return an estimate if we are in the wrong pipeline or the estimate is invalid
        if(isLocalizationPipeline && lastPoseEstimate.isValid)
            return Optional.of(lastPoseEstimate);

        else
            return Optional.empty();
    }

    public AprilTagTarget getAprilTagTarget() {
        if(!isLocalizationPipeline)
            return currentTarget;

        else
            throw new IllegalStateException();
    }

    public static final class AprilTagTarget implements Sendable {
        public double xOffset = 0;
        public double yOffset = 0;
        public double id;
        public double area = 0;
        public boolean isValid = false;

        public AprilTagTarget() {
            SendableRegistry.add(this, "April Tag Target");
        }

        public boolean isSpeakerTag() {
            return id == 4 || id == 7;
        }

        public boolean isAmpTag() {
            return id == 6 || id == 5;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("X Offset", () -> xOffset, null);
            builder.addDoubleProperty("Y Offset", () -> yOffset, null);
            builder.addDoubleProperty("ID", () -> id, null);
            builder.addDoubleProperty("Area", () -> area, null);
            builder.addBooleanProperty("Is Valid", () -> isValid, null);
        }
    }

    public static final class VisionPoseMeasurement implements Sendable {
        public Pose2d pose = new Pose2d();
        public double measurementTime = 0;
        public boolean isValid = false;

        public VisionPoseMeasurement() {
            SendableRegistry.add(this, "Vision Pose");
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Pose X", () -> pose.getX(), null);
            builder.addDoubleProperty("Pose Y", () -> pose.getY(), null);
            builder.addBooleanProperty("Is Valid", () -> isValid, null); 
        }
    }
}   
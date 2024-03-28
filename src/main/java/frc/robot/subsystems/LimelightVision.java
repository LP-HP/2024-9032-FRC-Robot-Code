package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelightutil.LimelightHelpers;
import frc.lib.limelightutil.LimelightHelpers.LimelightTarget_Fiducial;
import frc.lib.limelightutil.LimelightHelpers.PoseEstimate;

import static frc.robot.Constants.LimelightConstants.*;

public class LimelightVision extends SubsystemBase {
    private LimelightPoseEstimate currentPose = new LimelightPoseEstimate();
    private AprilTagTarget currentTarget = new AprilTagTarget();
    
    private boolean isLocalizationPipeline = startInLocalization;//Whether we are localizing or tracking a target

    private final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    public LimelightVision() {
        /* Add Telemetry */
        limelightTab.add(currentTarget)
            .withPosition(6, 0).withSize(2, 2);
        limelightTab.add(currentPose)
            .withPosition(8, 0).withSize(2, 2);
        limelightTab.addBoolean("Localization Pipeline", () -> isLocalizationPipeline)
            .withPosition(8, 3).withSize(2, 1);

        /* Add Pipeline Buttons */
        limelightTab.add("To Localization", runOnce(this::switchToLocalizationPipeline))
            .withPosition(0, 4).withSize(2, 1);
        limelightTab.add("To Target", runOnce(this::switchToTargetPipeline))
            .withPosition(2, 4).withSize(2, 1);
    }

    public void addCameraToTab(ShuffleboardTab tab, int col, int row, int size) {
        try {
            tab.addCamera("Limelight View", limelightName, "camera_server://" + limelightName)
                .withPosition(col, row).withSize(size, size);
        } catch (Exception e) {
            System.err.println("Limelight view already added!");
        }
    }

    @Override
    public void periodic() {
        if(isLocalizationPipeline) 
            currentPose.poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        boolean isValid = LimelightHelpers.getTV(limelightName);

        if(isValid) {
            LimelightTarget_Fiducial tag = LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials[0];

            currentTarget.xOffset = tag.tx;
            currentTarget.yOffset = tag.ty;
            currentTarget.area = tag.ta;
            currentTarget.id = tag.fiducialID;
            currentTarget.skew = tag.ts;

            currentTarget.distance = getDistanceFromYOffset(currentTarget.yOffset);

            currentTarget.isValid = currentTarget.distance < distanceCutoff;
        }

        else  
            currentTarget.isValid = false;
    }

    private double getDistanceFromYOffset(double yOffset) {
        return (tagHeight - mountingHeight) / Math.tan(Units.degreesToRadians(yOffset) + mountingAngle);
    }

    public void switchToTargetPipeline() {
        LimelightHelpers.setPipelineIndex(limelightName, targetPipelineID);

        isLocalizationPipeline = false;
    }

    public void switchToLocalizationPipeline() {
        LimelightHelpers.setPipelineIndex(limelightName, localizationPipelineID);

        /* Switch speaker tag target based on alliance */
        if(DriverStation.getAlliance().get().equals(Alliance.Red))
            LimelightHelpers.setPriorityTagID(limelightName, 4);
            
        else
            LimelightHelpers.setPriorityTagID(limelightName, 7);

        isLocalizationPipeline = true;
    }

    public boolean isTargetPipeline() {
        return !isLocalizationPipeline;
    }

    public Optional<PoseEstimate> getPoseEstimate() {
        PoseEstimate currentReading = currentPose.poseEstimate;
        /* Do not return an estimate if we are in the wrong pipeline or the estimate is invalid */
        if(isLocalizationPipeline 
            && currentReading != null 
            && currentReading.tagCount != 0 
            && !(currentReading.pose.getX() == 0.0 && currentReading.pose.getY() == 0.0)
        ) {
            return Optional.of(currentReading);
        }

        else
            return Optional.empty();
    }

    public AprilTagTarget getAprilTagTarget() {
        return currentTarget;
    }

    public static final class AprilTagTarget implements Sendable {
        public double xOffset = 0.0;
        public double yOffset = 0.0;
        public double id = 0.0;
        public double area = 0.0;
        public boolean isValid = false;
        public double distance = 0.0;
        public double skew = 0.0;

        public AprilTagTarget() {
            SendableRegistry.add(this, "April Tag Target");
        }

        public boolean isValidSpeakerTag() {
            return isValid && (id == 4 || id == 7);
        }

        public boolean isValidAmpTag() {
            return isValid && (id == 6 || id == 5);
        }

        public boolean isValidStageTag() {
            /* IDs 11-16 are stage tags */
            return isValid && id >= 11 && id <= 16;//TODO put in ids in pipeline
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("X Offset", () -> xOffset, null);
            builder.addDoubleProperty("Y Offset", () -> yOffset, null);
            builder.addDoubleProperty("ID", () -> id, null);
            builder.addDoubleProperty("Area", () -> area, null);
            builder.addBooleanProperty("Is Valid", () -> isValid, null);
            builder.addDoubleProperty("Distance", () -> distance, null);
            builder.addDoubleProperty("Skew", () -> skew, null);
        }
    }

    public static final class LimelightPoseEstimate implements Sendable {
        public PoseEstimate poseEstimate = new PoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0);

        public LimelightPoseEstimate() {
            SendableRegistry.add(this, "Limelight Pose Estimate");
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Pose X", () -> poseEstimate.pose.getX(), null);
            builder.addDoubleProperty("Pose Y", () -> poseEstimate.pose.getY(), null);
            builder.addDoubleProperty("Pose Heading", () -> poseEstimate.pose.getRotation().getDegrees(), null);
            builder.addDoubleProperty("Tag Count", () -> poseEstimate.tagCount, null);
            builder.addDoubleProperty("Tag Area", () -> poseEstimate.avgTagArea, null);
            builder.addDoubleProperty("Timestamp", () -> poseEstimate.timestampSeconds, null);
        }
    }
}   
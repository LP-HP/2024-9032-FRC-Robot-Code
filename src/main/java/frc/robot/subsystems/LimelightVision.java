package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelightutil.LimelightHelpers;
import frc.lib.limelightutil.LimelightHelpers.PoseEstimate;

import static frc.robot.Constants.LimelightConstants.*;

public class LimelightVision extends SubsystemBase {
    private PoseEstimate currentPose = new PoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0);
    private AprilTagTarget currentTarget = new AprilTagTarget();
    
    private final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    public LimelightVision() {
        /* Add Telemetry */
        ShuffleboardLayout poseLayout = limelightTab.getLayout("Pose", BuiltInLayouts.kList)
            .withPosition(8, 0).withSize(2, 2);
        poseLayout.addDouble("Pose X", () -> currentPose.pose.getX());
        poseLayout.addDouble("Pose Y", () -> currentPose.pose.getY());
        poseLayout.addDouble("Pose Heading", () -> currentPose.pose.getRotation().getDegrees());
        poseLayout.addDouble("Tag Count", () -> currentPose.tagCount);
        poseLayout.addDouble("Tag Area", () -> currentPose.avgTagArea);
        poseLayout.addDouble("Timestamp", () -> currentPose.timestampSeconds);

        ShuffleboardLayout targetLayout = limelightTab.getLayout("Target", BuiltInLayouts.kList)
            .withPosition(6, 0).withSize(2, 2);
        targetLayout.addDouble("X Offset", () -> currentTarget.xOffset);
        targetLayout.addDouble("Y Offset", () -> currentTarget.yOffset);
        targetLayout.addDouble("ID", () -> currentTarget.id);
        targetLayout.addDouble("Area", () -> currentTarget.area);
        targetLayout.addBoolean("Is Valid", () -> currentTarget.isValid);
        targetLayout.addDouble("Distance", () -> currentTarget.distance);
        targetLayout.addDouble("Skew", () -> currentTarget.skew);

        limelightTab.addBoolean("Localization Pipeline", this::isLocalizationPipeline)
            .withPosition(8, 3).withSize(2, 1);

        /* Add Pipeline Buttons */
        limelightTab.add("To Localization", runOnce(this::switchToLocalizationPipeline))
            .withPosition(0, 4).withSize(2, 1);
        limelightTab.add("To Target", runOnce(this::switchToTargetPipeline))
            .withPosition(2, 4).withSize(2, 1);

        if(startInLocalization)
            switchToLocalizationPipeline();
        
        else
            switchToTargetPipeline();
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
        if(isLocalizationPipeline()) 
            currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        boolean isValid = LimelightHelpers.getTV(limelightName);

        if(isValid) {
            currentTarget.xOffset = LimelightHelpers.getTX(limelightName);
            currentTarget.yOffset = LimelightHelpers.getTY(limelightName);
            currentTarget.area = LimelightHelpers.getTA(limelightName);
            currentTarget.id = LimelightHelpers.getFiducialID(limelightName);
            currentTarget.skew = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getRotation().getZ();//TODO enable 3d tracking / delete target pipeline

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
    }

    public void switchToLocalizationPipeline() {
        LimelightHelpers.setPipelineIndex(limelightName, localizationPipelineID);

        /* Switch speaker tag target based on alliance */
        if(DriverStation.getAlliance().get().equals(Alliance.Red))
            LimelightHelpers.setPriorityTagID(limelightName, 4);
            
        else
            LimelightHelpers.setPriorityTagID(limelightName, 7);
    }

    public boolean isTargetPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == targetPipelineID;
    }

    public boolean isLocalizationPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == localizationPipelineID;
    }

    public Optional<PoseEstimate> getPoseEstimate() {
        /* Do not return an estimate if we are in the wrong pipeline or the estimate is invalid */
        if(isLocalizationPipeline() 
            && currentPose != null 
            && currentPose.tagCount != 0 
            && !(currentPose.pose.getX() == 0.0 && currentPose.pose.getY() == 0.0)
        ) {
            return Optional.of(currentPose);
        }

        else
            return Optional.empty();
    }

    public AprilTagTarget getAprilTagTarget() {
        return currentTarget;
    }

    public static final class AprilTagTarget {
        public double xOffset = 0.0;
        public double yOffset = 0.0;
        public double id = 0.0;
        public double area = 0.0;
        public boolean isValid = false;
        public double distance = 0.0;
        public double skew = 0.0;

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
    }
}   
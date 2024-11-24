package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.LocalizationPhotonVisionConstants.*;

import java.util.Optional;

public class Localization{
    private final ShuffleboardTab localizationTab;
    private final Field2d[] fields = new Field2d[kNumberCameras];

    private final PhotonCamera[] cameras = new PhotonCamera[kNumberCameras];
    private final PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[kNumberCameras];

    private final SwerveDrivePoseEstimator swervePoseEstimator;
    
    public Localization(SwerveDrivePoseEstimator swervePoseEstimator) {  
        for(int i = 0; i < kNumberCameras; i++){
            cameras[i] = new PhotonCamera(kCameraNames[i]);
            photonPoseEstimators[i] = new PhotonPoseEstimator(
                kAprilTagFieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                cameras[i], 
                kRobotToCameraTransforms[i]
            );   

            fields[i] = new Field2d();
            /*cameras are 0 through n-1*/
        }

        localizationTab = Shuffleboard.getTab("Localization");
        
        this.swervePoseEstimator = swervePoseEstimator;
    }

    private void addCamerasToTab(ShuffleboardTab tab, int col, int row, int size) {//TODO review and implement this method
        /*telemetry*/
        try {
            /*change*/
            for(int i = 0; i < kNumberCameras; i++){
                localizationTab.add("Field", fields[i]).withPosition(0, i*4).withSize(8, 3);
            }
            
            /*localizationTab.addCamera("PhotonVision View", cameraName, "http://photonvision.local:1182/stream.mjpg").withPosition(col, row).withSize(size, size);*/
        } catch (Exception e) {
            System.err.println("Cameras already added!");
        }
    }
    
    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {//TODO review this method
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
        var t3d = target.getBestCameraToTarget();
        var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
        if (distance < smallestDistance)
            smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                    + kPoseAmbiguityOffset)
                    * kPoseAmbiguityMultiplier);
        //if there is 1 
        double confidenceMultiplier = Math.max(
            1,
            (Math.max(
                1,
                Math.max(0, smallestDistance - kNoisyDistanceMeters)
                    * kDistanceWeight)
                * poseAmbiguityFactor)
                / (1
                    + ((estimation.targetsUsed.size() - 1) * kTagPresenceWeight)));

        return kVisionStandardDeviations.times(confidenceMultiplier);
    }

    public void update(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
        /* Update each photonPoseEstimator and add the measurement to the swerve one */
        for(int i = 0; i < kNumberCameras; i++){
            Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimators[i].update();

            if (optionalEstimatedPose.isPresent()) {
                EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();        
                  
                swervePoseEstimator.addVisionMeasurement(
                    estimatedPose.estimatedPose.toPose2d(), 
                    estimatedPose.timestampSeconds, 
                    confidenceCalculator(estimatedPose)
                );

                fields[i].setRobotPose(estimatedPose.estimatedPose.toPose2d());
            }
        }

        swervePoseEstimator.update(gyroYaw, modulePositions); 
    }

    public void resetPosition(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions, Pose2d givenPose){
        swervePoseEstimator.resetPosition(gyroYaw, modulePositions, givenPose);
    }

    public Pose2d getRobotPose(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions){
        swervePoseEstimator.update(gyroYaw, modulePositions);//TODO Maybe don't update here 
        
        return swervePoseEstimator.getEstimatedPosition();
    }
}
   
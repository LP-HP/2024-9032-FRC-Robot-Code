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
    private ShuffleboardTab localizationTab;
    private final PhotonCamera[] cameras = new PhotonCamera[nCameras];
    
    private final PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[nCameras];
    private SwerveDrivePoseEstimator poseEstimator;
    
    private final Field2d[] fields = new Field2d[nCameras];
    public Localization(SwerveDrivePoseEstimator Estimator) {  
        for(int i = 0; i < nCameras; i++){
            cameras[i] = new PhotonCamera(cameraNames[i]);
        /*cameras are 0 through n-1*/
        }

        for(int i = 0; i < nCameras; i++){
            photonPoseEstimators[i] = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[i], robotToCam[i]);
        }
        for(int i = 0; i < nCameras; i++){
            fields[i] = new Field2d();
        }
        localizationTab = Shuffleboard.getTab("Localization");
        
        poseEstimator = Estimator;
    }

    public void addCamerasToTab(ShuffleboardTab tab, int col, int row, int size) {
        /*telemetry*/
        try {
            /*change*/
            for(int i = 0; i < nCameras; i++){
                localizationTab.add("Field", fields[i]).withPosition(0, i*4).withSize(8, 3);
            }
            
            /*localizationTab.addCamera("PhotonVision View", cameraName, "http://photonvision.local:1182/stream.mjpg").withPosition(col, row).withSize(size, size);*/
        } catch (Exception e) {
            System.err.println("Cameras already added!");
        }
    }
    
    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
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
                    + POSE_AMBIGUITY_SHIFTER)
                    * POSE_AMBIGUITY_MULTIPLIER);
        //if there is 1 
        double confidenceMultiplier = Math.max(
            1,
            (Math.max(
                1,
                Math.max(0, smallestDistance - NOISY_DISTANCE_METERS)
                    * DISTANCE_WEIGHT)
                * poseAmbiguityFactor)
                / (1
                    + ((estimation.targetsUsed.size() - 1) * TAG_PRESENCE_WEIGHT)));

        return VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }
    public void update(Rotation2d GyroYaw, SwerveModulePosition[] ModulePositions) {
        //do for all each
        for(int i = 0; i < nCameras; i++){
            Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimators[i].update();
            if (optionalEstimatedPose.isPresent()) {
                final EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();          
                poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds,confidenceCalculator(optionalEstimatedPose.get()));
                fields[i].setRobotPose(estimatedPose.estimatedPose.toPose2d());
            }
        }
        poseEstimator.update(GyroYaw, ModulePositions); 
        
    }
    public void resetPosition(Rotation2d gyroYaw, SwerveModulePosition[] ModulePositions, Pose2d givenPose){
        poseEstimator.resetPosition(gyroYaw, ModulePositions, givenPose);
    }
    public Pose2d getRobotPose(Rotation2d GyroYaw, SwerveModulePosition[] ModulePositions){
        poseEstimator.update(GyroYaw, ModulePositions); 
        return poseEstimator.getEstimatedPosition();
    }
}
   
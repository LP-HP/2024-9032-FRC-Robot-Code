package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelightutil.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;
import frc.robot.util.SparkMaxConstants.SparkMaxPIDConstants;

import static frc.robot.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator swerveOdometry;
    private final SwerveModule[] swerveMods;
    private final Pigeon2 gyro;

    private final Field2d field = new Field2d();
    private final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

    private Supplier<Optional<PoseEstimate>> visionSup = Optional::empty;

    private SimpleMotorFeedforward velocityFeedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);
    private SparkMaxPIDConstants velocityPID = drivePIDConstants;

    public Swerve() {
        gyro = new Pigeon2(gyroID);//Automatically calibrates

        swerveMods = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };

        /* 
         * The 1 second wait before setting absolute positions avoids a bug with initializing the motors
         * See: https://github.com/Team364/BaseFalconSwerve/issues/8 
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();//Set integrated encoders to the absolute positions using cancoders

        swerveOdometry = new SwerveDrivePoseEstimator(swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
        /* Set heading deviation high to only use gyro for heading */
        swerveOdometry.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.5, 0.5, 999999999)
        );

        /* Sets up pathplanner for auto path following */
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getSpeeds, 
                this::driveOpenLoopFromSpeeds,//TODO This should be closed loop
                new HolonomicPathFollowerConfig(
                    Constants.ClosedLoopConstants.translationPID,
                    Constants.ClosedLoopConstants.headingPID, 
                    maxSpeed, 
                    driveRadius,
                    new ReplanningConfig()), 
                /* Supplier for if the path should be mirrored for the red alliance - will always use blue for origin */
                () -> DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this);

        /* Show field view */
        swerveTab.add("Field", field)
            .withPosition(0, 0).withSize(8, 3);

        // /* Send pathplanner target pose to field view */
        // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //     field.getObject("target pose").setPose(pose);
        // });

        // /* Send pathplanner path to field view */
        // PathPlannerLogging.setLogActivePathCallback((poses) -> {
        //     field.getObject("path").setPoses(poses);
        // });

        /* Add Telemetry */
        swerveTab.addDouble("Pose X", () -> getPose().getX())
            .withPosition(0, 4).withSize(1, 1);
        swerveTab.addDouble("Pose Y", () -> getPose().getY())
            .withPosition(1,4).withSize(1, 1);
        swerveTab.addDouble("Pose Heading", () -> getPose().getRotation().getDegrees())
            .withPosition(2, 4).withSize(1, 1);

        /* Add Constants */
        GenericEntry kS = swerveTab.add("kS", velocityFeedforward.ks)
            .withPosition(3, 4).withSize(1, 1)
            .getEntry();
        GenericEntry kV = swerveTab.add("kV", velocityFeedforward.kv)
            .withPosition(4, 4).withSize(1, 1)
            .getEntry();
        GenericEntry kP = swerveTab.add("kP", velocityPID.kP())
            .withPosition(5, 4).withSize(1, 1)
            .getEntry();
        GenericEntry kD = swerveTab.add("kD", velocityPID.kD())
            .withPosition(6, 4).withSize(1, 1)
            .getEntry();
        swerveTab.add(runOnce(() -> updateConstantsFromDashboard(kS, kV, kP, kD)).withName("Update Constants"))
            .withPosition(7, 4).withSize(1, 1);
    }

    public Command getVisionLocalizationAuto(String autoName, Supplier<Optional<PoseEstimate>> visionSup) {
        return addOptionalVisionPoseSupplier(visionSup)
            .andThen(AutoBuilder.buildAuto(autoName));
    }

    private void updateConstantsFromDashboard(GenericEntry kS, GenericEntry kV, GenericEntry kP, GenericEntry kD) {
        velocityFeedforward = new SimpleMotorFeedforward(kS.getDouble(0), kV.getDouble(0));
        velocityPID = new SparkMaxPIDConstants(kP.getDouble(0), 0.0, kD.getDouble(0), 0.0, -1.0, 1.0);

        for(SwerveModule mod : swerveMods) {
            mod.updateVelocityConstants(velocityFeedforward, velocityPID);
        }
    }

    public void driveClosedLoopFromSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getNumber()], false);
        }
    }

    public void driveClosedLoop(Translation2d translation, double rotation) {
        driveClosedLoopFromSpeeds(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    }

    public void driveOpenLoopFromSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getNumber()], true);
        }
    }

    public void driveOpenLoop(Translation2d translation, double rotation, boolean fieldCentric) {
        ChassisSpeeds speeds = fieldCentric ? 
            /* For field centric driving */
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation, 
                getGyroYaw()
                )
            /* Otherwise robot relative */
            : new ChassisSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation
            );

        /* Kinematics wants module angles in the range (-180, 180] */ 
        SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getNumber()], true);
        }
    }     

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public Command resetOdometryCommand(Supplier<Pose2d> poseSup) {
        return runOnce(() -> resetOdometry(poseSup.get()));
    }

    private void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods) {
            states[mod.getNumber()] = mod.getState();
        }

        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods) {
            positions[mod.getNumber()] = mod.getPosition();
        }

        return positions;
    }

    public ChassisSpeeds getSpeeds() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro() {
        gyro.reset();
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Command addOptionalVisionPoseSupplier(Supplier<Optional<PoseEstimate>> poseSupplier) {
        return runOnce(() -> visionSup = poseSupplier);
    }

    private void updateVisionLocalization(PoseEstimate poseEstimate) {
        /* Multiple targets detected means a lower standard deviation */
        if(poseEstimate.tagCount >= 2) {
            double poseDifference = getPose().getTranslation().getDistance(poseEstimate.pose.getTranslation());

            /* Discard any measurements that are too far from previous measurements */
            if(poseDifference < 1.0) 
                swerveOdometry.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);

            else 
                System.err.println("Discarded pose estimate | pose " + poseEstimate.pose + " | dif. " + poseDifference);
        }     
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());  

        /* Only update vision if an update is provided */
        if(!visionSup.get().isEmpty()) 
            updateVisionLocalization(visionSup.get().get());
        
        field.setRobotPose(getPose());//Update field view
    }
}
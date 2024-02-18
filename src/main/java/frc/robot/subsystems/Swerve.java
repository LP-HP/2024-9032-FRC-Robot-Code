package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightVision.VisionPoseMeasurement;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator swerveOdometry;
    private final SwerveModule[] swerveMods;
    private final AHRS gyro;

    private final Field2d field = new Field2d();
    private final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

    private Supplier<Optional<VisionPoseMeasurement>> visionSup = Optional::empty;

    public Swerve() {
        gyro = new AHRS(gyroPort);//Automatically calibrates

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

        /* Sets up pathplanner for auto path following */
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getSpeeds, 
                this::driveClosedLoopFromSpeeds,
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
            .withPosition(1, 4).withSize(10, 10);

        /* Send pathplanner target pose to field view */
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        /* Send pathplanner path to field view */
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        /* Add Telemetry */
        swerveTab.addDouble("Pose X", () -> getPose().getX())
            .withPosition(1, 1).withSize(1, 1);
        swerveTab.addDouble("Pose Y", () -> getPose().getY())
            .withPosition(2, 1).withSize(1, 1);
        swerveTab.addDouble("Pose Heading", () -> getPose().getRotation().getDegrees())
            .withPosition(3, 1).withSize(1, 1);
    }

    public void driveClosedLoopFromSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(desiredStates[mod.getNumber()], false);
        }
    }

    public void driveClosedLoop(Translation2d translation, double rotation) {
        driveClosedLoopFromSpeeds(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
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

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods){
            states[mod.getNumber()] = mod.getState();
        }

        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods){
            positions[mod.getNumber()] = mod.getPosition();
        }

        return positions;
    }

    public ChassisSpeeds getSpeeds() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    private Rotation2d getGyroYaw() {
        return invertGyro ? Rotation2d.fromDegrees(-gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void addOptionalVisionPoseSupplier(Supplier<Optional<VisionPoseMeasurement>> poseSupplier) {
        visionSup = poseSupplier;
    }

    private void updateVisionLocalization(Pose2d visionPose, double time) {
        Transform2d poseDifference = visionPose.minus(swerveOdometry.getEstimatedPosition());

        /* Only add the vision update if it is within the tolerance - prevents vision noise */
        if(Math.abs(poseDifference.getX()) < Constants.VisionConstants.visionPoseTolerance
        && Math.abs(poseDifference.getY()) < Constants.VisionConstants.visionPoseTolerance) {
             swerveOdometry.addVisionMeasurement(visionPose, time);
        } 

        else
            System.err.println("Discarded vision measurement " + visionPose);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());  

        /* Only update vision if an update is provided */
        if(!visionSup.get().isEmpty()) {
            VisionPoseMeasurement measurement = visionSup.get().get();

            updateVisionLocalization(measurement.pose, measurement.measurementTime);
        }

        field.setRobotPose(getPose());//Update field view
    }
}
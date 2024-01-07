package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private AHRS gyro;

    private final Field2d field = new Field2d();

    public Swerve() {
        gyro = new AHRS(Constants.Swerve.gyroPort);
        gyro.calibrate();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* The 1 second wait before setting absolute positions avoids a bug with initializing the motors
         * Stolen from: https://github.com/Team364/BaseFalconSwerve/issues/8 
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();//Set integrated encoders to the absolute positions using cancoders

        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        AutoBuilder.configureHolonomic(//TODO check pathplannerlib
                this::getPose,
                this::resetOdometry,
                this::getSpeeds, 
                this::driveAuto,
                new HolonomicPathFollowerConfig(
                    new PIDConstants(Constants.AutoConstants.kPTranslation, 0, Constants.AutoConstants.kDTranslation),
                    new PIDConstants(Constants.AutoConstants.kPRotation, 0, Constants.AutoConstants.kDRotation), 
                    Constants.Swerve.maxSpeed, 
                    Constants.Swerve.driveRadius,
                    new ReplanningConfig()), 
                this);

        SmartDashboard.putData("Field", field);//Show field view

        //Send pathplanner pose to field view
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        //Send pathplanner target pose to field view
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        //Send pathplanner path to field view
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
    }

    public void driveAuto(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getNumber()], false);
        }
    }

    public void driveClosedLoop(Translation2d translation, double rotation) {
        driveAuto(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    }

    public void driveOpenLoop(Translation2d translation, double rotation, boolean fieldCentric) {
        SwerveModuleState[] desiredStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldCentric ? ChassisSpeeds.fromFieldRelativeSpeeds(//For field relative driving
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getGyroYaw()
                                )
                                : new ChassisSpeeds(//Otherwise robot relative
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.getNumber()], true);
        }
    }     

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.getNumber()] = mod.getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.getNumber()] = mod.getPosition();
        }

        return positions;
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    private Rotation2d getGyroYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {//TODO change dashboard
        swerveOdometry.update(getGyroYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods) {//Send swerve module telemetry
            int modNum = mod.getNumber();

            SmartDashboard.putNumber("Mod " + modNum + " Cancoder Angle", mod.getCanCoderAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + modNum + " Integrated Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + modNum + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        
        Pose2d currentPose = swerveOdometry.getPoseMeters();

        SmartDashboard.putNumber("Pose X", currentPose.getX());//Send odometry telemetry
        SmartDashboard.putNumber("Pose Y", currentPose.getY());
        SmartDashboard.putNumber("Pose Rot", currentPose.getRotation().getDegrees());
    }
}
package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.swerveutil.SwerveModuleConstants;
import frc.robot.util.SparkMaxConstants;
import frc.robot.util.SparkMaxConstants.ControlMode;
import frc.robot.util.SparkMaxConstants.SparkMaxPIDConstants;

public final class Constants {
    public static final int driveControllerPort = 0;

    public static final class TeleopConstants {
        public static final double stickDeadband = 0.05;
        public static final boolean isFieldCentric = true;
        /* Meters per Second */
        public static final double joystickToSpeedConversionFactor = 1;
        /* Radians per Second */
        public static final double joystickToAngularVelocityConversionFactor = Math.PI;
        /* Meters per Second Squared */
        public static final double accelerationLimit = 16.0;//TODO tune
        public static final double decelerationLimit = -16.0;//TODO tune
    }

    public static final class VisionConstants {
        public static final double visionPoseTolerance = 1.0;//TODO tune for localization (in meters)

        public static final String limelightName = "9032Limelight";//TODO set name
        public static final int targetPipelineID = 1;
        public static final int localizationPipelineID = 0;//TODO make sure this aligns with the limelight config
    }

    /* Using CANIds 13-14 - 2 motors */
    public static final class IntakeConstants {//TODO tune
        /* Intake Arm */
        public static final double armSetpointTolerance = 0.1;
        public static final SparkMaxPIDConstants intakeArmPID = new SparkMaxPIDConstants(
            1.0, 
            0.0, 
            0.0, 
            0.0
        );
        public static final SparkMaxConstants intakeArmConstants = new SparkMaxConstants(
            13,
            "Intake Arm",
            ControlMode.position,
            intakeArmPID,
            40,
            false,
            IdleMode.kBrake,
            12,
            1.0 / 60.0//TODO make sure this works
        );
        public static final boolean invertAbsoluteEncoder = false;
        /* Arm Positions */
        public static final double armPositionGround = -0.029761908575892;
        public static final double armPositionPassthrough = 0.0;
        public static final double armPositionAmp = -0.33372887969017;
        public static final double armPositionStorage = 0.0;

        /* Intake Flywheel */
        public static final SparkMaxConstants intakeFlywheelConstants = new SparkMaxConstants(
            14,
            "Intake Flywheels",
            ControlMode.percentOutput,
            null,
            40,
            false,
            IdleMode.kBrake,
            12,
            1.0
        );
        /* Flyhweel Powers */
        public static final double intakePower = -0.2;
        public static final double outtakeAmpPower = 0.6;
        public static final double outtakeToShooterPower = -0.1;

        /* Sensors */
        public static final int beamBreakPort = 0;
    }

    /* Using CANIds 15-19 - 5 motors */
    public static final class ShooterConstants {//TODO tune
        /* Shooter Arm */
        public static final double armSetpointTolerance = 0.1;
        public static final SparkMaxPIDConstants shooterArmPID = new SparkMaxPIDConstants(
            0.0, 
            0.0, 
            0.0, 
            0.0
        );
        public static final SparkMaxConstants shooterArmConstants = new SparkMaxConstants(
            15,
            "Shooter Arm",
            ControlMode.positionLeader,
            shooterArmPID,
            60,
            false,
            IdleMode.kBrake,
            12,
            1.0 / 60.0//TODO make sure this works
        );
        public static final SparkMaxConstants shooterArmFolllowerConstants = new SparkMaxConstants(
            16,
            "Shooter Arm Follower",
            ControlMode.percentOutput,
            shooterArmPID,
            60,
            false,
            IdleMode.kBrake,
            12,
            1.0 / 60.0
        );
        public static final boolean invertArmFollower = true;//TODO INVERT OR NO
        public static final boolean invertAbsoluteEncoder = true;
        /* Arm Positions */
        public static final double armPositionPassthrough = 100.0;
        public static final double armPositionStorage = 80.0;
        /* Key - Target Y Offset : Value - Arm Position */
        public static final InterpolatingDoubleTreeMap armPosLookupTableFromTargetY = new InterpolatingDoubleTreeMap();
        static {
            armPosLookupTableFromTargetY.put(0.0, 3.0);
            armPosLookupTableFromTargetY.put(1.0, 4.0);
        }

        /* Shooter Flywheels */
        public static final double shooterFlywheelVelocityTolerance = 0.1;//TODO UNITSSSSS
        public static final SparkMaxPIDConstants shooterFlywheelPID = new SparkMaxPIDConstants(
            0.0, 
            0.0, 
            0.0, 
            0.0
        );
        public static final SparkMaxConstants shooterFlywheelConstants = new SparkMaxConstants(
            17,
            "Shooter Flywheel",
            ControlMode.velocityLeader,
            shooterFlywheelPID,
            60,
            false,
            IdleMode.kBrake,
            12,
            60.0
        );
        public static final SparkMaxConstants shooterFlywheelFolllowerConstants = new SparkMaxConstants(
            18,
            "Shooter Flywheel Follower",
            ControlMode.percentOutput,
            shooterFlywheelPID,
            60,
            false,
            IdleMode.kBrake,
            12,
            60.0
        );
        public static final boolean invertFlywheelFollower = true;//TODO INVERT OR NO

        /* Storage Motor */
        public static final SparkMaxConstants shooterStorageConstants = new SparkMaxConstants(
            19,
            "Shooter Storage",
            ControlMode.percentOutput,
            shooterFlywheelPID,
            20,
            false,
            IdleMode.kBrake,
            12,
            1.0
        );
        public static final double storageMotorPowerReceiving = 0.5;
        public static final double storageMotorPowerToFlywheels = 1;

        /* Sensors */
        public static final int beamBreakPort = 1;
    }

    /* Using CANIds 1-12 - 8 motors and 4 cancoders */
    public static final class SwerveConstants {
        public static final boolean invertGyro = true;
        public static final SPI.Port gyroPort = SPI.Port.kMXP;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.5); 
        public static final double wheelBase = Units.inchesToMeters(24.5); 
        /* Distance from the center of the robot to the furthest module */
        public static final double driveRadius = Math.sqrt(
            Math.pow(wheelBase / 2.0, 2.0) + 
            Math.pow(trackWidth / 2.0, 2.0));
        public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;
        public static final double driveMotorMaxRPM = 5676.0;

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 6.75;
        public static final double angleGearRatio = 150.0 / 7.0;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = true;
        public static final boolean driveMotorInvert = false;

        /* Angle Encoder Inverts */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        public static final boolean integratedEncoderInvert = false;

        /* Swerve Voltage Compensation */
        public static final int voltageComp = 12;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 60;

        /* Angle Motor PID Values */
        public static final SparkMaxPIDConstants anglePIDConstants = new SparkMaxPIDConstants(//TODO Tune - needed for teleop
            0.06, 
            0.0, 
            0.0, 
            0.0
        );

        /* Drive Motor PID Values */
        public static final SparkMaxPIDConstants drivePIDConstants = new SparkMaxPIDConstants( //TODO: This must be tuned to specific robot - needed for closed loop
            0.05, 
            0.0, 
            0.0, 
            0.0
        );

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.0; //TODO: This must be tuned to specific robot - needed for auto
        public static final double driveKV = 0.0;
        public static final double driveKA = 0.0;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionFactor = wheelCircumference / driveGearRatio;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Theoretical Max Speed - Meters per Second */
        public static final double maxSpeed = (driveMotorMaxRPM / 60.0) * (1.0 / driveGearRatio) * wheelCircumference;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final SparkMaxConstants driveMotorConstants = new SparkMaxConstants(
                1,
                "Drive Front Left",
                ControlMode.velocityControlWithPositionData,
                drivePIDConstants,
                driveContinuousCurrentLimit,
                driveMotorInvert,
                driveNeutralMode,
                voltageComp,
                driveConversionFactor
            );
            public static final SparkMaxConstants angleMotorConstants = new SparkMaxConstants(
                2,
                "Angle Front Left",
                ControlMode.position,
                anglePIDConstants,
                angleContinuousCurrentLimit,
                angleMotorInvert,
                angleNeutralMode,
                voltageComp,
                angleConversionFactor
            );
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-94.482);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorConstants, angleMotorConstants, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final SparkMaxConstants driveMotorConstants = new SparkMaxConstants(
                3,
                "Drive Front Right",
                ControlMode.velocityControlWithPositionData,
                drivePIDConstants,
                driveContinuousCurrentLimit,
                driveMotorInvert,
                driveNeutralMode,
                voltageComp,
                driveConversionFactor
            );
            public static final SparkMaxConstants angleMotorConstants = new SparkMaxConstants(
                4,
                "Angle Front Right",
                ControlMode.position,
                anglePIDConstants,
                angleContinuousCurrentLimit,
                angleMotorInvert,
                angleNeutralMode,
                voltageComp,
                angleConversionFactor
            );
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-40.430);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorConstants, angleMotorConstants, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final SparkMaxConstants driveMotorConstants = new SparkMaxConstants(
                5,
                "Drive Back Left",
                ControlMode.velocityControlWithPositionData,
                drivePIDConstants,
                driveContinuousCurrentLimit,
                driveMotorInvert,
                driveNeutralMode,
                voltageComp,
                driveConversionFactor
            );
            public static final SparkMaxConstants angleMotorConstants = new SparkMaxConstants(
                6,
                "Angle Back Left",
                ControlMode.position,
                anglePIDConstants,
                angleContinuousCurrentLimit,
                angleMotorInvert,
                angleNeutralMode,
                voltageComp,
                angleConversionFactor
            );
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(57.920);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorConstants, angleMotorConstants, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final SparkMaxConstants driveMotorConstants = new SparkMaxConstants(
                7,
                "Drive Back Right",
                ControlMode.velocityControlWithPositionData,
                drivePIDConstants,
                driveContinuousCurrentLimit,
                driveMotorInvert,
                driveNeutralMode,
                voltageComp,
                driveConversionFactor
            );
            public static final SparkMaxConstants angleMotorConstants = new SparkMaxConstants(
                8,
                "Angle Back Right",
                ControlMode.position,
                anglePIDConstants,
                angleContinuousCurrentLimit,
                angleMotorInvert,
                angleNeutralMode,
                voltageComp,
                angleConversionFactor
            );
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-106.700);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorConstants, angleMotorConstants, canCoderID, angleOffset);
        }
    }

    public static final class ClosedLoopConstants { 
        /* PID Constants for Path Following */
        public static final double kPTranslation = 1.0; // TODO: TUNE for auto pathplanner
        public static final double kDTranslation = 0.0;

        public static final double kPRotation = 1.0;
        public static final double kDRotation = 0.0;

         /* PID Constants for Rotation to a Target */
        public static final double kPRotationTarget = 1.0;//TODO tune and test
        public static final double kDRotationTarget = 0.0;
    }
}
package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.util.SparkMaxConstants;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.SparkMaxConstants.ControlMode;
import frc.robot.util.SparkMaxConstants.SparkMaxPIDConstants;

/* 
 * ...
 */
public final class Constants {
    public static final int driveControllerPort = 0;
    public static final int mechanismControllerPort = 1;

    public static final boolean burnFlash = false;
    public static final boolean configMotors = false;

    public static final double passthroughRecoveryWait = 3.0;

    public static final class LEDConstants {
        public static final int ledPWMPort = 0;
        public static final int ledStripLength = 61;
    }

    public static final class TeleopConstants {
        public static final double stickDeadband = 0.02;
        public static final boolean isFieldCentric = true;
        /* Meters per Second */
        public static final double joystickToSpeedConversionFactor = SwerveConstants.maxSpeed;
        /* Radians per Second */
        public static final double joystickToAngularVelocityConversionFactor = 2 * Math.PI;
    }

    public static final class LimelightConstants {
        public static final String limelightName = "limelight";
        public static final int targetPipelineID = 0;
        public static final int localizationPipelineID = 1;
        public static final boolean startInLocalization = false;

        /* Distance Constants */
        public static final double tagHeight = Units.inchesToMeters(57.25);
        public static final double mountingHeight = Units.inchesToMeters(12.75);
        public static final double mountingAngle = Units.degreesToRadians(30.0);

        /* Cutoff for target validity */
        public static final double distanceCutoff = 6.0;
    }

    public static final class PhotonvisionConstants {
        public static final String cameraName = "IntakeCamera";

        /* Distance Constants */
        public static final double targetHeight = Units.inchesToMeters(1.0);
        public static final double cameraHeight = Units.inchesToMeters(25.0);
        public static final double mountingAngle = Units.degreesToRadians(-18.0); 
    }

    /* Using CANIds 13-14 - 2 motors */
    public static final class IntakeConstants {
        /* Intake Arm */
        public static final double armSetpointTolerance = 5.0;
        public static final SparkMaxPIDConstants intakeArmPID = new SparkMaxPIDConstants(
            0.02, 
            0.0, 
            0.0, 
            0.0,
            -0.5,
            0.5
        );
        public static final SparkMaxConstants intakeArmConstants = new SparkMaxConstants(
            13,
            "Intake Arm",
            ControlMode.position,
            intakeArmPID,
            40,
            true,
            IdleMode.kBrake,
            12,
            360.0 / 60.0
        );
        /* Arm Positions */
        public static final double armPositionStarting = 229.0;
        public static final double armPositionGround = 26.5;
        public static final double armPositionPassthrough = 227.0;
        public static final double armPositionEject = 100.0;

        /* Intake Flywheel */
        public static final SparkMaxConstants intakeRollerConstants = new SparkMaxConstants(
            14,
            "Intake Roller",
            ControlMode.percentOutput,
            null,
            40,
            false,
            IdleMode.kBrake,
            12,
            1.0
        );
        public static final double shotWaitTime = 0.25;
        /* Flyhweel Powers */
        public static final double intakePower = -1.0;
        public static final double ejectPower = 0.5;
        public static final double transferToShooterPower = 0.4;

        /* Sensors */
        public static final int beamBreakPort = 0;

        public static final int ultrasonicPingPort = 2;
        public static final int ultrasonicEchoPort = 3;
        public static final int medianFilterSize = 5;
        /* In Inches */
        public static final double closeToObstacleDistance = 2.5;
    }

    /* Using CANIds 15-19 - 5 motors */
    public static final class ShooterConstants {
        /* Shooter Arm */
        public static final double armSetpointTolerance = 1.0;
        public static final double minArmSetpoint = 95.0;
        public static final double maxArmSetpoint = 180.0;
        public static final SparkMaxPIDConstants shooterArmPID = new SparkMaxPIDConstants(
            0.05, 
            0.0, 
            0.01, 
            0.0,
            -0.4,
            0.5
        );
        public static final SparkMaxConstants shooterArmConstants = new SparkMaxConstants(
            15,
            "Shooter Arm",
            ControlMode.positionLeader,
            shooterArmPID,
            60,
            true,
            IdleMode.kBrake,
            12,
            360.0 / (280 / 3.0)
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
            360.0 / (280 / 3.0)
        );
        public static final boolean invertArmFollower = true;
        public static final boolean invertAbsoluteEncoder = true;
        public static final double absoluteEncoderConversionFactor = 360.0;
        public static final double absoluteEncoderOffset = 267.23;

        /* Arm Positions */
        public static final double armPositionPassthrough = 133.0;
        public static final double armPositionUnderStage = 120.0;
        public static final double armPositionUp = 160.0;
        public static final double armPositionAmp = 145.0;
        public static final double armPositionTrap = 147.0;
        public static final double armPositionShuttle = 140.0;

        /* Auto Arm Aiming */
        /* Key - Distance : Value - Arm Position */
        public static final InterpolatingDoubleTreeMap distanceToArmPosTable = new InterpolatingDoubleTreeMap();
        static {
            distanceToArmPosTable.put(1.105, 149.0);
            distanceToArmPosTable.put(1.296, 148.0);
            distanceToArmPosTable.put(1.452, 146.5);
            distanceToArmPosTable.put(1.636, 145.0);
            distanceToArmPosTable.put(1.97, 140.5);
            distanceToArmPosTable.put(2.12, 139.0);
            distanceToArmPosTable.put(2.291, 136.5);
            distanceToArmPosTable.put(2.577, 135.5);
            distanceToArmPosTable.put(2.77, 135.0);
            distanceToArmPosTable.put(2.942, 132.0);
            distanceToArmPosTable.put(3.202, 130.0);
            distanceToArmPosTable.put(3.427, 129.0);
            distanceToArmPosTable.put(3.638, 128.0);
            distanceToArmPosTable.put(3.972, 127.5);
            distanceToArmPosTable.put(4.27, 127.0);
            distanceToArmPosTable.put(4.72, 124.55);
        }
        public static final double distanceVelocityCompAmt = 0.0;//TODO test

        /* Shooter Flywheels */
        public static final double flywheelVelocityTolerance = 1.0;
        public static final int flywheelSupplyCurrentLimit = 60;
        public static final double flywheelSupplyTimeThreshold = 0.2;
        public static final double flywheelkP = 0.1;
        public static final double flywheelkD = 0.0;
        public static final double flywheelkV = 0.117;
        public static final double flywheelkS = 0.0;

        public static final int leftFlywheelMotorID = 17;
        public static final InvertedValue leftFlywheelInvert = InvertedValue.Clockwise_Positive;

        public static final int rightFlywheelMotorID = 18;
        public static final InvertedValue rightFlywheelInvert = InvertedValue.CounterClockwise_Positive;

        public static final double speakerShotWaitTime = 0.5;
        public static final double ampShotWaitTime = 1.0;
        public static final double minFlywheelSetpoint = 0.0;
        public static final double maxFlywheelSetpoint = 100.0;
        public static final double flywheelAmpSetpoint = 13.0;
        public static final double flywheelTrapSetpoint = 35.0;

        /* Storage Motor */
        public static final SparkMaxConstants shooterStorageConstants = new SparkMaxConstants(
            19,
            "Shooter Storage",
            ControlMode.percentOutput,
            null,
            20,
            true,
            IdleMode.kBrake,
            12,
            1.0
        );
        public static final double storageMotorPowerReceiving = 0.2;
        public static final double storageMotorPowerToFlywheels = 0.75;
        public static final double storageMotorPowerToAmp = 0.3;

        /* Sensors */
        public static final int beamBreakPort = 2;
    }

    /* Using CANIds 20-21 - 2 motors */
    public static final class ClimberConstants {
        /* Left Climber */
        public static final SparkMaxConstants leftClimberConstants = new SparkMaxConstants(
            20, 
            "Left Climber", 
            ControlMode.position, 
            null, 
            60, 
            false, 
            IdleMode.kBrake, 
            12, 
            1.0
        );
        /* Right Climber */
        public static final SparkMaxConstants rightClimberConstants = new SparkMaxConstants(
            21, 
            "Right Climber", 
            ControlMode.position, 
            null, 
            60, 
            true, 
            IdleMode.kBrake, 
            12, 
            1.0
        );

        public static final double maxHeight = 61.0;//TODO set this
        public static final double minHeight = -1.0;
    }

    /* Using CANIds 1-12 - 8 motors and 4 cancoders */
    public static final class SwerveConstants {
        public static final boolean invertGyro = true;
        public static final Port gyroPort = Port.kMXP;

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

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Voltage Compensation */
        public static final int voltageComp = 12;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 60;

        /* Angle Motor PID Values */
        public static final SparkMaxPIDConstants anglePIDConstants = new SparkMaxPIDConstants(
            0.06, 
            0.0, 
            0.0, 
            0.0,
            -1.0,
            1.0
        );

        /* Drive Motor PID Values */
        public static final SparkMaxPIDConstants drivePIDConstants = new SparkMaxPIDConstants( //TODO: This must be tuned to specific robot - needed for closed loop
            0.75, 
            0.0, 
            0.0, 
            0.0,
            -1.0,
            1.0
        );

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.0; //TODO: This must be tuned to specific robot - needed for auto
        public static final double driveKV = 3.0;
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
                41,
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
        public static final PIDConstants translationPID = new PIDConstants(// TODO: TUNE for auto pathplanner
            6.0, 
            0.0, 
            0.0
        ); 
        public static final PIDConstants headingPID = new PIDConstants(// TODO: TUNE for auto pathplanner
            6.0, 
            0.0, 
            0.0
        ); 

        /* Constants for aiming at the speaker while moving */
        public static final double kPSpeakerRotation = 0.14;
        public static final double kDSpeakerRotation = 0.01;

        public static final double xOffsetVelocityCompAmt = 0.0;//TODO test
        public static final double skewCompAmtCutoff = 0.15;
        public static final double skewCompAmt = -5.0;

        /* Constants for note alignment */        
        public static final double kPNoteRotation = 0.12;
        public static final double kDNoteRotation = 0.01;

        public static final double kPNoteDistance = 1.75;
        public static final double kDNoteDistance = 0.01;
        
        public static final double maxNoteDrivingSpeed = 3.0;
        public static final int cycleAmtSinceNoteSeenCutoff = 5;
        public static final double noteXSetpoint = 0.58;

        /* Constants for stage alignment */
        public static final double kPStageRotation = 8.0;
        public static final double kDStageRotation = 0.01;
        public static final double stageRotationTolerance = 0.015;

        public static final double kPStageDistance = 2.0;
        public static final double kDStageDistance = 0.01;
        public static final double stageDistanceTolerance = Units.inchesToMeters(1.0);
        public static final double stageDistanceSetpoint = 0.93;

        public static final double kPStageY = 0.13;
        public static final double kDStageY = 0.02;
        public static final double stageYTolerance = 1.0;

        /* April Tag Constants */
        public static final int cycleAmtSinceAprilTagSeenCutoff = 5;
    }

    public static final class AutoConstants { 
        /* Seconds */
        public static final double notePickupTimeout = 5.0;

        public static final double shootVelocity = 95.0;
    }
}
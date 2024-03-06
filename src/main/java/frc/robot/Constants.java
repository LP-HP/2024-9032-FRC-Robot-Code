package frc.robot;

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
    public static final boolean burnFlash = false;

    public static final class TeleopConstants {
        public static final double stickDeadband = 0.02;
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
        public static final String limelightName = "limelight";//TODO set name to 9032Limeligh
        public static final int targetPipelineID = 0;
        public static final int localizationPipelineID = 1;//TODO make sure this aligns with the limelight config
        public static final boolean startInLocalization = false;

        /* Distance Constants */
        public static final double tagHeight = Units.inchesToMeters(60.0);
        public static final double mountingHeight = Units.inchesToMeters(10.0);
        public static final double mountingAngle = Units.degreesToRadians(18.0);
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
        public static final boolean invertAbsoluteEncoder = false;
        public static final double absoluteEncoderConversionFactor = 360.0;
        public static final double absoluteEncoderOffset = 98.1747508;
        /* Arm Positions */
        public static final double armPositionGround = 26.5;
        public static final double armPositionPassthrough = 225.0;
        public static final double armPositionAmp = 160.0;

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
        public static final double shotWaitTime = 0.25;
        /* Flyhweel Powers */
        public static final double intakePower = -0.3;
        public static final double outtakeAmpPower = 0.7;
        public static final double transferToShooterPower = 0.4;

        /* Sensors */
        public static final int beamBreakPort = 0;
    }

    /* Using CANIds 15-19 - 5 motors */
    public static final class ShooterConstants {
        /* Shooter Arm */
        public static final double armSetpointTolerance = 1.0;
        public static final double minSetpoint = 90.0;
        public static final double maxSetpoint = 180.0;
        public static final SparkMaxPIDConstants shooterArmPID = new SparkMaxPIDConstants(
            0.05, 
            0.0, 
            0.01, 
            0.0,
            -0.3,
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
        public static final double armPositionPassthrough = 130.0;
        public static final double armPositionStorage = 140.0;
        /* Key - Distance : Value - Arm Position */
        public static final InterpolatingDoubleTreeMap distanceToArmPosTable = new InterpolatingDoubleTreeMap();
        static {
            distanceToArmPosTable.put(-3.34, 128.0);
            distanceToArmPosTable.put(-0.12, 129.0);
            distanceToArmPosTable.put(4.58, 132.0);
            distanceToArmPosTable.put(15.51, 142.0);
        }

        /* Shooter Flywheels */
        public static final double flywheelVelocityTolerance = 50.0;
        public static final SparkMaxPIDConstants shooterFlywheelPID = new SparkMaxPIDConstants(
            0.0003, 
            0.0, 
            0.001, 
            0.000175,
            -1.0,
            1.0
        );
        public static final SparkMaxConstants shooterFlywheelConstants = new SparkMaxConstants(
            17,
            "Shooter Flywheel",
            ControlMode.velocityLeader,
            shooterFlywheelPID,
            60,
            false,
            IdleMode.kCoast,
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
            IdleMode.kCoast,
            12,
            60.0
        );
        public static final boolean invertFlywheelFollower = true;
        public static final double shotWaitTime = 1.0;//TODO change

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
        public static final double storageMotorPowerReceiving = 0.15;
        public static final double storageMotorPowerToFlywheels = 0.75;

        /* Sensors */
        public static final int beamBreakPort = 2;
    }

    /* Using CANIds 20-21 - 2 motors */
    public static final class ClimberConstants {
        /* Left Climber */
        public static final SparkMaxConstants leftClimberConstants = new SparkMaxConstants(
            20, 
            "Left Climber", 
            ControlMode.percentOutput, 
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
            ControlMode.percentOutput, 
            null, 
            60, 
            false, 
            IdleMode.kBrake, 
            12, 
            1.0
        );
    }

    /* Using CANIds 1-12 - 8 motors and 4 cancoders */
    public static final class SwerveConstants {
        public static final boolean invertGyro = true;
        public static final Port gyroPort = Port.kUSB;

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
        public static final SparkMaxPIDConstants anglePIDConstants = new SparkMaxPIDConstants(//TODO Tune - needed for teleop
            0.06, 
            0.0, 
            0.0, 
            0.0,
            -1.0,
            1.0
        );

        /* Drive Motor PID Values */
        public static final SparkMaxPIDConstants drivePIDConstants = new SparkMaxPIDConstants( //TODO: This must be tuned to specific robot - needed for closed loop
            0.05, 
            0.0, 
            0.0, 
            0.0,
            -1.0,
            1.0
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
        public static final PIDConstants translationPID = new PIDConstants(// TODO: TUNE for auto pathplanner
            0.0, 
            0.0, 
            0.0
        ); 
        public static final PIDConstants headingPID = new PIDConstants(// TODO: TUNE for auto pathplanner
            0.0, 
            0.0, 
            0.0
        ); 

        /* PID Constants for rotation and movement to a vision target */
        public static final double kPRotationTarget = 0.1;//TODO tune and test
        public static final double kPTranslationTarget = 0.01;//TODO tune and test
        public static final double rotationSetpointTolerance = 1.0;
        public static final double translationSetpointTolerance = 0.5;
    }

    public static final class AutoConstants { //TODO tune
        /* Seconds */
        public static final double notePickupTimeout = 2.0;

        public static final double armPosNote1 = 10.0;
        public static final double shooterVelocityNote1 = 10.0;
        public static final double armPosNote2 = 10.0;
        public static final double shooterVelocityNote2 = 10.0;
    }
}
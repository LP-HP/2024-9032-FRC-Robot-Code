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

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final int driveControllerPort = 0;

    public static final class VisionConstants {
        public static final double visionPoseTolerance = 1;//TODO tune for localization (in meters)

        public static final String limelightName = "9032Limelight";//TODO set name
        public static final int targetPipelineID = 1;
        public static final int localizationPipelineID = 0;//TODO make sure this aligns with the limelight config
    }

    public static final class IntakeConstants {//TODO tune
        /* Intake Arm */
        public static final int armMotorID = 13;
        public static final double armPositionGround = 0;
        public static final double kPArm = 0;
        public static final double kDArm = 0;

        /* Arm and Flywheel Motor  */
        public static final int motorCurrentLimit = 80;
        public static final int motorVoltageComp = 12;

        /* Intake Flywheel */
        public static final int intakeFlywheelMotorID = 14;
        public static final double intakeVelocity = 1;
        public static final double kPIntake = 0;
        public static final double kDIntake = 0;
    }

     public static final class ShooterConstants {//TODO tune
        /* Shooter Arm */
        public static final int armMotorID = 15;
        public static final double kPArm = 0;
        public static final double kDArm = 0;
        public static final double armSetpointTolerance = 0.5;
        /* Key - Target Area : Value - Arm Position */
        public static final InterpolatingDoubleTreeMap armPosLookupTableFromArea = new InterpolatingDoubleTreeMap();
        static {
            armPosLookupTableFromArea.put(0.0, 3.0);
            armPosLookupTableFromArea.put(1.0, 4.0);
        }

        /* Arm and Flywheel Motor  */
        public static final int motorCurrentLimit = 80;
        public static final int motorVoltageComp = 12;

        /* Shooter Flywheel */
        public static final int shooterFlywheelMotorID = 16;
        public static final double kPShooter = 0;
        public static final double kDShooter = 0;
    }

    public static final class SwerveConstants {
        public static final boolean invertGyro = false; //TODO ensure Gyro works
        public static final SPI.Port gyroPort = SPI.Port.kMXP;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot - needed for teleop
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot - needed for teleop
        public static final double driveRadius = Units.inchesToMeters(2);//TODO Distance from the center of the robot to the furthest module - needed for auto
        public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

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
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;//TODO tune if needed 
        public static final int driveContinuousCurrentLimit = 80;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;//TODO Tune - needed for teleop
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKF = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot - needed for closed loop
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0; //TODO: This must be tuned to specific robot - needed for auto
        public static final double driveKV = 0;
        public static final double driveKA = 0;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        /* Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot - needed for teleop
        /* Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot - needed for teleop

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot - needed for teleop
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot - needed for teleop
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot - needed for teleop
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot - needed for teleop
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class ClosedLoopConstants { 
        /* PID Constants for Path Following */
        public static final double kPTranslation = 1; // TODO: TUNE for auto pathplanner
        public static final double kDTranslation = 0;

        public static final double kPRotation = 1;
        public static final double kDRotation = 0;

         /* PID Constants for Rotation to a Target */
        public static final double kPRotationTarget = 1;//TODO tune and test
        public static final double kDRotationTarget = 0;
    }
}
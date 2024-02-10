package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.SwerveModuleConstants;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
    private int moduleNumber;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
  
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;
  
    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState currentState = getState();
       
        /* Reverse the direction if needed to avoid rotating more than 90 degrees - works since PID wrapping is enabled */
        desiredState = SwerveModuleState.optimize(desiredState, currentState.angle);

        setAngle(desiredState);
        setSpeed(desiredState, currentState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, SwerveModuleState currentState, boolean isOpenLoop) {
        if (isOpenLoop) {
            /* Scale speed by cosine of angle error, which slows down movement perpendicular to the desired direction of travel
             * Smoothes driving when modules change directions
             * See: https://github.com/wpilibsuite/allwpilib/pull/5758
             */
            desiredState.speedMetersPerSecond *= desiredState.angle.minus(currentState.angle).getCos();

            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;

            driveMotor.set(percentOutput);
        } 
        else {
            driveController.setReference(//Set the closed loop velocity controller with a feedforward component
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public int getNumber() {
        return moduleNumber;
    }

    private void setAngle(SwerveModuleState desiredState) {
        /* Prevent rotating module if speed is less then 1% to prevent jittering */
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getIntegratedAngle() {
        return Rotation2d.fromDegrees(Constants.SwerveConstants.integratedEncoderInvert 
            /* Invert and put the angle in the range (-180, 180] */
            ? MathUtil.inputModulus(-integratedAngleEncoder.getPosition(), -180.0, 180.0)
            /* Put the angle in the range (-180, 180] */
            : MathUtil.inputModulus(integratedAngleEncoder.getPosition(), -180.0, 180.0)
            );
    }

    public Rotation2d getCanCoderAngle() {
        /* In the range (-180, 180] */
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        /* Add the offset to zero the module and put in the range (-180, 180] */
        double absolutePosition = MathUtil.inputModulus(getCanCoderAngle().minus(angleOffset).getDegrees(), -180.0, 180.0);

        integratedAngleEncoder.setPosition(absolutePosition);

        /* Move to zeroed position */
        angleController.setReference(absolutePosition, ControlType.kPosition);
    }

    private void configAngleEncoder() {      
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

        /* Put in the range (-180, 180] and invert if needed */
        cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
        
        angleEncoder.getConfigurator().apply(cancoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
        angleController.setP(Constants.SwerveConstants.angleKP);
        angleController.setI(Constants.SwerveConstants.angleKI);
        angleController.setD(Constants.SwerveConstants.angleKD);
        angleController.setFF(Constants.SwerveConstants.angleKF);
        /* Wrap in the range (-180, 180] */
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(180.0);
        angleController.setPositionPIDWrappingMinInput(-180.0);
        angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        angleMotor.burnFlash();
    }

    private void configDriveMotor() {        
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
        driveController.setP(Constants.SwerveConstants.angleKP);
        driveController.setI(Constants.SwerveConstants.angleKI);
        driveController.setD(Constants.SwerveConstants.angleKD);
        driveController.setFF(Constants.SwerveConstants.angleKF);
        driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getIntegratedAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getIntegratedAngle());
    }
}
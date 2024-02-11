package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.swerveutil.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.util.SparkMaxWrapper;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public class SwerveModule {
    private int moduleNumber;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private SparkMaxWrapper angleMotor;
    private SparkMaxWrapper driveMotor;
  
    private CANcoder angleEncoder;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    private ShuffleboardTab swerveModuleTab;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new SparkMaxWrapper(moduleConstants.angleMotorConstants);
        /* Wrap in the range (-180, 180] */
        angleMotor.configPIDWrapping(-180.0, 180.0);
        angleMotor.config();

        /* Drive Motor Config */
        driveMotor = new SparkMaxWrapper(moduleConstants.driveMotorConstants);
        driveMotor.relativeEncoder.setPosition(0.0);
        driveMotor.config();

        lastAngle = getState().angle;

        /* Add Telemetry */
        swerveModuleTab = Shuffleboard.getTab("Module " + moduleNumber);
        swerveModuleTab.add(angleMotor).withPosition(1, 1).withSize(2, 4);
        swerveModuleTab.add(driveMotor).withPosition(4, 1).withSize(2, 4);
        swerveModuleTab.add(angleEncoder).withPosition(7, 1).withSize(2, 4);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState currentState = getState();
       
        /* Reverse the direction if needed to avoid rotating more than 90 degrees - works since PID wrapping is enabled */
        desiredState = SwerveModuleState.optimize(desiredState, currentState.angle);

        /* 
         * Scale speed by cosine of angle error, which slows down movement perpendicular to the desired direction of travel
         * Smoothes driving when modules change directions
         * See: https://github.com/wpilibsuite/allwpilib/pull/5758
         */
        double cosineScalar = desiredState.angle.minus(currentState.angle).getCos();
        if(cosineScalar < 0.0) {
            System.err.println("Negative cosine scalar!");
        }
        desiredState.speedMetersPerSecond *= cosineScalar;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;

            driveMotor.set(percentOutput);
        } 
        else {
            /* Set the closed loop velocity controller with a feedforward component */
            driveMotor.setClosedLoopTarget(desiredState.speedMetersPerSecond, feedforward.calculate(desiredState.speedMetersPerSecond));
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

        angleMotor.setClosedLoopTarget(angle.getDegrees());
        lastAngle = angle;
    }

    private Rotation2d getIntegratedAngle() {
        /* Put the angle in the range (-180, 180] */
        return Rotation2d.fromDegrees(MathUtil.inputModulus(angleMotor.relativeEncoder.getPosition(), -180.0, 180.0));
    }

    private Rotation2d getCanCoderAngle() {
        /* In the range (-180, 180] */
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        /* Add the offset to zero the module and put in the range (-180, 180] */
        double absolutePosition = MathUtil.inputModulus(getCanCoderAngle().minus(angleOffset).getDegrees(), -180.0, 180.0);

        angleMotor.relativeEncoder.setPosition(absolutePosition);

        /* Move to zeroed position */
        angleMotor.setClosedLoopTarget(absolutePosition);
    }

    private void configAngleEncoder() {      
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

        /* Put in the range (-180, 180] and invert if needed */
        cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
        
        angleEncoder.getConfigurator().apply(cancoderConfig);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.relativeEncoder.getVelocity(), getIntegratedAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.relativeEncoder.getPosition(), getIntegratedAngle());
    }
}
package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.CTREModuleState;
import frc.lib.swerveutil.SwerveModuleConstants;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
  
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;
  
    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
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

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKF);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }
}
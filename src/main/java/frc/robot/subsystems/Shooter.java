package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax armMotor;    
    private SparkPIDController armController;
    private RelativeEncoder armEncoder;
    private double armSetpoint;

    private CANSparkMax shooterFlywheelMotor;
    private SparkPIDController shooterController;

    private CANSparkMax passthroughStorageMotor;

    private final DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreakPort);

    public Shooter() {
        armMotor = new CANSparkMax(Constants.ShooterConstants.armMotorID, MotorType.kBrushless);
        armController = armMotor.getPIDController();
        configArmMotor();

        shooterFlywheelMotor = new CANSparkMax(Constants.ShooterConstants.shooterFlywheelMotorID, MotorType.kBrushless);
        shooterController = shooterFlywheelMotor.getPIDController();
        configShooterMotor();

        passthroughStorageMotor = new CANSparkMax(Constants.ShooterConstants.storageMotorID, MotorType.kBrushless);
        configStorageMotor();

        armEncoder = armMotor.getEncoder();
        armEncoder.setPositionConversionFactor(Constants.ShooterConstants.armEncoderConversionFactor);
        /* Reset the relative encoder to the absolute encoder value */
        armEncoder.setPosition(armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }

    private void configArmMotor() {
        armMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly);
        armMotor.setSmartCurrentLimit(Constants.ShooterConstants.neoV1CurrentLimit);
        armMotor.setIdleMode(IdleMode.kBrake);
        armController.setP(Constants.ShooterConstants.kPArm);
        armController.setD(Constants.ShooterConstants.kDArm);
        armMotor.enableVoltageCompensation(Constants.ShooterConstants.motorVoltageComp);
        armMotor.burnFlash();
    }

    private void configShooterMotor() {
        shooterFlywheelMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(shooterFlywheelMotor, Usage.kVelocityOnly);
        shooterFlywheelMotor.setSmartCurrentLimit(Constants.ShooterConstants.neoV1CurrentLimit);
        shooterFlywheelMotor.setIdleMode(IdleMode.kBrake);
        shooterController.setP(Constants.ShooterConstants.kPShooter);
        shooterController.setD(Constants.ShooterConstants.kDShooter);
        shooterFlywheelMotor.enableVoltageCompensation(Constants.ShooterConstants.motorVoltageComp);
        shooterFlywheelMotor.burnFlash();
    }

    private void configStorageMotor() {
        passthroughStorageMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kMinimal);
        passthroughStorageMotor.setSmartCurrentLimit(Constants.ShooterConstants.neo550CurrentLimit);
        passthroughStorageMotor.setIdleMode(IdleMode.kBrake);
        passthroughStorageMotor.enableVoltageCompensation(Constants.ShooterConstants.motorVoltageComp);
        passthroughStorageMotor.burnFlash();
    }

    public boolean isBeamBreakTriggered() {
        return !beamBreak.get();//TODO true or false?
    }

    public Command enableStorageMotorReceiving() {
        return runOnce(() -> passthroughStorageMotor.set(Constants.ShooterConstants.storageMotorPowerReceiving));
    }

    private Command enableStorageMotorToFlywheels() {
        return runOnce(() -> passthroughStorageMotor.set(Constants.ShooterConstants.storageMotorPowerToFlywheels));
    }

    public Command disableStorageMotor() {
        return runOnce(() -> passthroughStorageMotor.set(0));
    }

    private Command waitForShooterVelocity(double velocity) {
        return new FunctionalCommand(
        /* Sets the target velocity at the start */
        () -> shooterController.setReference(velocity, ControlType.kVelocity), 
        () -> {}, 
        (unused) -> {}, 
        /* We are finished if the flywheel velocity is within our tolerance */
        () -> Math.abs(shooterFlywheelMotor.getEncoder().getVelocity() - velocity) < Constants.ShooterConstants.shooterFlywheelVelocityTolerance, 
        this);
    }

    private Command disableShooterFlywheel() {
        return runOnce(() -> shooterController.setReference(0, ControlType.kVelocity));
    }

    public Command setShooterVelocityThenWaitThenDisable(double velocity, double waitTime) {
        return waitForShooterVelocity(velocity)
           .andThen(enableStorageMotorToFlywheels())
           .andThen(Commands.waitSeconds(waitTime))
           .andThen(disableShooterFlywheel())
           .andThen(disableStorageMotor());
    }

    /* Just sets the target */
    private void setArmTargetPosition(double position) {
        armSetpoint = position;

        armController.setReference(armSetpoint, ControlType.kPosition); 
    }

    public Command setToPassthroughPosition() {
        return runOnce(() -> setArmTargetPosition(Constants.ShooterConstants.armPositionPassthrough)); 
    }

    public Command setToStoragePosition() {
        return runOnce(() -> setArmTargetPosition(Constants.ShooterConstants.armPositionStorage)); 
    }

    /* Sets the target and wait until it is achieved */
    private Command moveArmToTargetPosition(double position) { 
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> setArmTargetPosition(position),
        () -> {},
        (unused) -> {},
        /* We are finished if the arm position is within our tolerance */
        () -> Math.abs(armMotor.getEncoder().getPosition() - armSetpoint) < Constants.ShooterConstants.armSetpointTolerance,
        this);
    }   

    public Command moveArmToPassthroughPosition() {
        return moveArmToTargetPosition(Constants.ShooterConstants.armPositionPassthrough);
    }

    public Command moveArmToPositionFromTargetY(DoubleSupplier targetYSup) { 
        /* Sets the target position to an interpolated value from the lookup table */
        return moveArmToTargetPosition(Constants.ShooterConstants.armPosLookupTableFromTargetY.get(targetYSup.getAsDouble()));
    }   

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Arm Position Relative", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter Arm Setpoint", armSetpoint);
        SmartDashboard.putNumber("Shooter Flywheel Velocity", shooterFlywheelMotor.getEncoder().getVelocity());
    }
}
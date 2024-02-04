package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax armMotor;    
    private SparkPIDController armController;
    private RelativeEncoder armEncoder;

    private CANSparkMax intakeFlywheelMotor;

    private final DigitalInput beamBreak = new DigitalInput(Constants.IntakeConstants.beamBreakPort);

    public Intake() {
        armMotor = new CANSparkMax(Constants.IntakeConstants.armMotorID, MotorType.kBrushless);
        armController = armMotor.getPIDController();
        armEncoder = armMotor.getEncoder();
        configArmMotor();

        intakeFlywheelMotor = new CANSparkMax(Constants.IntakeConstants.intakeFlywheelMotorID, MotorType.kBrushless);
        configIntakeMotor();

         /* Reset the relative encoder to the absolute encoder value */
        // armEncoder.setPosition(armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());//TODO 
    }

    private void configArmMotor() {
        armMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly);
        armMotor.setSmartCurrentLimit(Constants.IntakeConstants.motorCurrentLimit);
        armMotor.setIdleMode(IdleMode.kBrake);
        armEncoder.setPositionConversionFactor(Constants.IntakeConstants.armEncoderConversionFactor);
        armController.setP(Constants.IntakeConstants.kPArm);
        armController.setD(Constants.IntakeConstants.kDArm);
        armMotor.enableVoltageCompensation(Constants.IntakeConstants.motorVoltageComp);
        armMotor.burnFlash();
    }

    private void configIntakeMotor() {
        intakeFlywheelMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeFlywheelMotor, Usage.kVelocityOnly);
        intakeFlywheelMotor.setSmartCurrentLimit(Constants.IntakeConstants.motorCurrentLimit);
        intakeFlywheelMotor.setIdleMode(IdleMode.kBrake);
        intakeFlywheelMotor.enableVoltageCompensation(Constants.IntakeConstants.motorVoltageComp);
        intakeFlywheelMotor.burnFlash();
    }

    public boolean isBeamBreakTriggered() {
        return !beamBreak.get();//TODO true or false?
    }

    /* Just sets the target */
    private void setArmTargetPosition(double position) {
        armController.setReference(position, ControlType.kPosition);
    }

    public Command setToGroundPosition() {
        return runOnce(() -> setArmTargetPosition(Constants.IntakeConstants.armPositionGround));
    }

    public Command setToPassthroughPosition() {
        return runOnce(() -> setArmTargetPosition(Constants.IntakeConstants.armPositionPassthrough));
    }

    public Command setToStoragePosition() {
        return runOnce(() -> setArmTargetPosition(Constants.IntakeConstants.armPositionStorage));
    }

    /* Sets the target and wait until it is achieved */
    private Command moveToTargetPosition(double position) {
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> setArmTargetPosition(position),
        () -> {},
        (unused) -> {},
         /* We are finished if the arm position is within our tolerance */
        () -> Math.abs(armEncoder.getPosition() - position) < Constants.IntakeConstants.armSetpointTolerance,
        this);
    } 

    public Command moveToAmpPosition() {
        return moveToTargetPosition(Constants.IntakeConstants.armPositionAmp);
    }

    public Command moveToPassthroughPosition() {
        return moveToTargetPosition(Constants.IntakeConstants.armPositionPassthrough);
    }

    private void setIntakePower(double power) {
        intakeFlywheelMotor.set(power);
    }
    
    public Command enableIntake() {
        return runOnce(() -> setIntakePower(Constants.IntakeConstants.intakePower));
    }

    public Command disableIntake() {
        return runOnce(() -> setIntakePower(0));
    }

    public Command shootIntoAmp() {
        return runOnce(() -> setIntakePower(Constants.IntakeConstants.outtakeAmpPower));
    }

    public Command shootIntoShooter() {
        return runOnce(() -> setIntakePower(Constants.IntakeConstants.outtakeToShooterPower));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Arm Position Relative", armEncoder.getPosition());
        SmartDashboard.putNumber("Intake Arm Speed", armMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake Arm Position Absolute", armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
        SmartDashboard.putNumber("Intake Flywheel Velocity", intakeFlywheelMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Beam Break Triggered", isBeamBreakTriggered());
    }
}
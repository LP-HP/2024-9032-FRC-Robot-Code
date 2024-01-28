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
    private SparkPIDController intakeController;

    private final DigitalInput beamBreak = new DigitalInput(Constants.IntakeConstants.beamBreakPort);

    public Intake() {
        armMotor = new CANSparkMax(Constants.IntakeConstants.armMotorID, MotorType.kBrushless);
        armController = armMotor.getPIDController();
        armEncoder = armMotor.getEncoder();
        configArmMotor();

        intakeFlywheelMotor = new CANSparkMax(Constants.IntakeConstants.intakeFlywheelMotorID, MotorType.kBrushless);
        intakeController = intakeFlywheelMotor.getPIDController();
        configIntakeMotor();

         /* Reset the relative encoder to the absolute encoder value */
        armEncoder.setPosition(armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }

    private void configArmMotor() {
        armMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly);
        armMotor.setSmartCurrentLimit(Constants.IntakeConstants.motorCurrentLimit);
        armMotor.setIdleMode(IdleMode.kBrake);
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
        intakeController.setP(Constants.IntakeConstants.kPIntake);
        intakeController.setD(Constants.IntakeConstants.kDIntake);
        intakeFlywheelMotor.enableVoltageCompensation(Constants.IntakeConstants.motorVoltageComp);
        intakeFlywheelMotor.burnFlash();
    }

    public boolean isBeamBreakTriggered() {
        return !beamBreak.get();//TODO true or false?
    }

    /* Just sets the target */
    public Command setToGroundPosition() {
        return runOnce(() -> armController.setReference(Constants.IntakeConstants.armPositionGround, ControlType.kPosition));
    }

    public Command setToAmpPosition() {
        return runOnce(() -> armController.setReference(Constants.IntakeConstants.armPositionAmp, ControlType.kPosition));
    }

    /* Sets the target and wait until it is achieved */
    public Command moveToPassthroughPosition() {
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> armController.setReference(Constants.IntakeConstants.armPositionPassthrough, ControlType.kPosition),
        () -> {},
        (unused) -> {},
         /* We are finished if the arm position is within our tolerance */
        () -> Math.abs(armEncoder.getPosition() - Constants.IntakeConstants.armPositionPassthrough) < Constants.IntakeConstants.armSetpointTolerance,
        this);
    } 

    public Command enableIntake() {
        return runOnce(() -> intakeController.setReference(Constants.IntakeConstants.intakeVelocity, ControlType.kVelocity));
    }

    public Command disableIntake() {
        return runOnce(() -> intakeController.setReference(0, ControlType.kVelocity));
    }

    public Command shootIntoAmp() {
        return runOnce(() -> intakeController.setReference(Constants.IntakeConstants.outtakeAmpVelocity, ControlType.kVelocity));
    }

    public Command shootIntoShooter() {
        return runOnce(() -> intakeController.setReference(Constants.IntakeConstants.outtakeToShooterVelocity, ControlType.kVelocity));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Flywheel Velocity", intakeFlywheelMotor.getEncoder().getVelocity());
    }
}
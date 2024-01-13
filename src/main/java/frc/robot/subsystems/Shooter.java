package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax armMotor;    
    private SparkPIDController armController;
    private double armSetpoint;

    private CANSparkMax shooterFlywheelMotor;
    private SparkPIDController shooterController;

    public Shooter() {
        armMotor = new CANSparkMax(Constants.ShooterConstants.armMotorID, MotorType.kBrushless);
        armController = armMotor.getPIDController();
        configArmMotor();

        shooterFlywheelMotor = new CANSparkMax(Constants.ShooterConstants.shooterFlywheelMotorID, MotorType.kBrushless);
        shooterController = shooterFlywheelMotor.getPIDController();
        configShooterMotor();
    }

    private void configArmMotor() {
        armMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly);
        armMotor.setSmartCurrentLimit(Constants.ShooterConstants.motorCurrentLimit);
        armMotor.setIdleMode(IdleMode.kBrake);
        armController.setP(Constants.ShooterConstants.kPArm);
        armController.setD(Constants.ShooterConstants.kDArm);
        armMotor.enableVoltageCompensation(Constants.ShooterConstants.motorVoltageComp);
        armMotor.getEncoder().setPosition(0);//TODO use absolute encoder
        armMotor.burnFlash();
    }

    private void configShooterMotor() {
        shooterFlywheelMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(shooterFlywheelMotor, Usage.kVelocityOnly);
        shooterFlywheelMotor.setSmartCurrentLimit(Constants.ShooterConstants.motorCurrentLimit);
        shooterFlywheelMotor.setIdleMode(IdleMode.kBrake);
        shooterController.setP(Constants.ShooterConstants.kPShooter);
        shooterController.setD(Constants.ShooterConstants.kDShooter);
        shooterFlywheelMotor.enableVoltageCompensation(Constants.ShooterConstants.motorVoltageComp);
        shooterFlywheelMotor.burnFlash();
    }

    public Command setShooterVelocity(double velocity) {
        return runOnce(() -> shooterController.setReference(velocity, ControlType.kVelocity));
    }

    public Command disableShooterFlywheel() {
        return runOnce(() -> shooterController.setReference(0, ControlType.kVelocity));
    }
    
    /* Just sets the target */
    public Command setArmTargetPosition(double position) {
        return runOnce(() -> {
            armSetpoint = position;

            armController.setReference(armSetpoint, ControlType.kPosition); 
        });
    }

    /* Sets the target and wait until it is achieved */
    public Command moveArmToPositionFromArea(DoubleSupplier areaSup) { 
        return new FunctionalCommand(
        /* Sets target the position at the start to an interpolated value from the lookup table */
        () -> {
            armSetpoint = Constants.ShooterConstants.armPosLookupTableFromArea.get(areaSup.getAsDouble());

            armController.setReference(armSetpoint, ControlType.kPosition);
        },
        () -> {},
        (unused) -> {},
        /* We are finished if the arm position is within our tolerance */
        () -> Math.abs(armMotor.getEncoder().getPosition() - armSetpoint) < Constants.ShooterConstants.armSetpointTolerance,
        this);
    }   

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter Arm Setpoint", armSetpoint);
        SmartDashboard.putNumber("Shooter Flywheel Velocity", shooterFlywheelMotor.getEncoder().getVelocity());
    }
}
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax armMotor;    
    private SparkPIDController armController;

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

    public Command setArmPosition(double position) {
        return runOnce(() -> armController.setReference(position, ControlType.kPosition));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter Flywheel Velocity", shooterFlywheelMotor.getEncoder().getVelocity());
    }
}
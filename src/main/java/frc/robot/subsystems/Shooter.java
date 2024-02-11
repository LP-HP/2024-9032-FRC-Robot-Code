package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SparkMaxWrapper;

public class Shooter extends SubsystemBase {
    private SparkMaxWrapper armMotor;    
    private SparkMaxWrapper armMotorFollower;    

    private SparkMaxWrapper flywheelMotor;
    private SparkMaxWrapper flywheelMotorFollower;

    private SparkMaxWrapper passthroughStorageMotor;

    private final DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreakPort);

    public Shooter() {
        armMotor = new SparkMaxWrapper(Constants.ShooterConstants.shooterArmConstants);
        armMotor.configAbsoluteEncoder(Constants.ShooterConstants.invertAbsoluteEncoder);
        armMotor.config();

        armMotorFollower = new SparkMaxWrapper(Constants.ShooterConstants.shooterArmFolllowerConstants);
        armMotorFollower.follow(armMotor, Constants.ShooterConstants.invertArmFollower);
        armMotorFollower.config();

        flywheelMotor = new SparkMaxWrapper(Constants.ShooterConstants.shooterFlywheelConstants);
        flywheelMotor.config();

        flywheelMotorFollower = new SparkMaxWrapper(Constants.ShooterConstants.shooterFlywheelFolllowerConstants);
        flywheelMotorFollower.follow(flywheelMotor, Constants.ShooterConstants.invertFlywheelFollower);
        flywheelMotorFollower.config();

        passthroughStorageMotor = new SparkMaxWrapper(Constants.ShooterConstants.shooterStorageConstants);
        passthroughStorageMotor.config();

         /* Wait for the encoder to initialize before setting to absolute */
        Timer.delay(1.0);

         /* Reset the relative encoder to the absolute encoder value */
        armMotor.relativeEncoder.setPosition(armMotor.getAbsolutePosition());

        SmartDashboard.putData(armMotor);
        SmartDashboard.putData(flywheelMotor);
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
        () -> flywheelMotor.setClosedLoopTarget(velocity), 
        () -> {}, 
        (unused) -> {}, 
        /* We are finished if the flywheel velocity is within our tolerance */
        () -> Math.abs(flywheelMotor.relativeEncoder.getVelocity() - velocity) < Constants.ShooterConstants.shooterFlywheelVelocityTolerance, 
        this);
    }

    private Command disableShooterFlywheel() {
        return runOnce(() -> flywheelMotor.setClosedLoopTarget(0));
    }

    public Command setShooterVelocityThenWaitThenDisable(double velocity, double waitTime) {
        return waitForShooterVelocity(velocity)
           .andThen(enableStorageMotorToFlywheels())
           .andThen(Commands.waitSeconds(waitTime))
           .andThen(disableShooterFlywheel())
           .andThen(disableStorageMotor());
    }

    public Command setToPassthroughPosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(Constants.ShooterConstants.armPositionPassthrough)); 
    }

    public Command setToStoragePosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(Constants.ShooterConstants.armPositionStorage)); 
    }

    /* Sets the target and wait until it is achieved */
    private Command moveArmToTargetPosition(double position) { 
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> armMotor.setClosedLoopTarget(position),
        () -> {},
        (unused) -> {},
        /* We are finished if the arm position is within our tolerance */
        () -> Math.abs(armMotor.relativeEncoder.getPosition() - position) < Constants.ShooterConstants.armSetpointTolerance,
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
        SmartDashboard.putBoolean("Shooter Beam Break Triggered", isBeamBreakTriggered());
    }
}
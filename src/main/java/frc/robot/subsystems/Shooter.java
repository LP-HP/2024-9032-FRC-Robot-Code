package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final SparkMaxWrapper armMotor;    
    private final SparkMaxWrapper armMotorFollower;    

    private final SparkMaxWrapper flywheelMotor;
    private final SparkMaxWrapper flywheelMotorFollower;

    private final SparkMaxWrapper passthroughStorageMotor;

    private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

    private final DigitalInput beamBreak = new DigitalInput(beamBreakPort);

    public Shooter() { 
        armMotor = new SparkMaxWrapper(shooterArmConstants);
        armMotor.configAbsoluteEncoder(invertAbsoluteEncoder, absoluteEncoderConversionFactor, absoluteEncoderOffset);
        armMotor.config();

        armMotorFollower = new SparkMaxWrapper(shooterArmFolllowerConstants);
        armMotorFollower.follow(armMotor, invertArmFollower);
        armMotorFollower.config();

        flywheelMotor = new SparkMaxWrapper(shooterFlywheelConstants);
        flywheelMotor.config();

        flywheelMotorFollower = new SparkMaxWrapper(shooterFlywheelFolllowerConstants);
        flywheelMotorFollower.follow(flywheelMotor, invertFlywheelFollower);
        flywheelMotorFollower.config();

        passthroughStorageMotor = new SparkMaxWrapper(shooterStorageConstants);
        passthroughStorageMotor.config();

         /* Wait for the encoder to initialize before setting to absolute */
        Timer.delay(1.0);

         /* Reset the relative encoder to the absolute encoder value */
        armMotor.relativeEncoder.setPosition(armMotor.getAbsolutePosition());

        /* Add Telemetry */
        shooterTab.add(armMotor)
            .withPosition(0, 0).withSize(2, 2);
        shooterTab.add(flywheelMotor)
            .withPosition(3, 0).withSize(2, 2);
        shooterTab.add(passthroughStorageMotor)
            .withPosition(0, 3).withSize(2, 2);
        shooterTab.addBoolean("Has Note", this::hasNote)
            .withPosition(6, 0).withSize(2, 1);
        shooterTab.addBoolean("Arm At Setpoint", this::armAtSetpoint)
            .withPosition(6, 1).withSize(2, 1);
        shooterTab.addBoolean("Flywheels At Setpoint", this::flywheelsAtSetpoint)
            .withPosition(6, 2).withSize(2, 1);

        /* Prevent moving to a previous setpoint */
        reset();
    }

    public boolean hasNote() {
        return !beamBreak.get();
    }

    public Command enableStorageMotorReceiving() {
        return runOnce(() -> passthroughStorageMotor.set(storageMotorPowerReceiving));
    }

    private Command enableStorageMotorToFlywheels() {
        return runOnce(() -> passthroughStorageMotor.set(storageMotorPowerToFlywheels));
    }

    private Command waitForShooterVelocity(double velocity) {
        return new FunctionalCommand(
        /* Sets the target velocity at the start */
        () -> flywheelMotor.setClosedLoopTarget(velocity), 
        () -> {}, 
        (unused) -> {}, 
        /* We are finished if the flywheel velocity is within our tolerance */
        this::flywheelsAtSetpoint, 
        this);
    }

    private Command disableShooterFlywheels() {
        return runOnce(() -> { 
            flywheelMotor.setClosedLoopTarget(0.0);
            passthroughStorageMotor.set(0.0);
        });
    }

    public Command shootSequence(double velocity) {
        return waitForShooterVelocity(velocity)
           .andThen(enableStorageMotorToFlywheels())
           .andThen(Commands.waitSeconds(shotWaitTime))
           .andThen(disableShooterFlywheels())
           .andThen(setToStoragePosition());
    }

    /* Sets the target and disables the storage motor (non-blocking) */
    private void setTargetPosition(double position) {
        armMotor.setClosedLoopTarget(position);
        passthroughStorageMotor.set(0.0); 
    }

    public Command setToStoragePosition() {
        return runOnce(() -> setTargetPosition(armPositionStorage)); 
    }

    public Command setToAutoPosition(double position) {
        return runOnce(() -> setTargetPosition(position));
    }

    /* Sets the target and wait until it is achieved */
    private Command moveToTargetPosition(double position) { 
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> setTargetPosition(position),
        () -> {},
        (unused) -> {},
        /* We are finished if the arm position is within our tolerance */
        this::armAtSetpoint,
        this);
    }   

    public Command moveToPassthroughPosition() {
        return moveToTargetPosition(armPositionPassthrough);
    }

    /* Moves to the target position from a vision target y offset */
    public Command moveToTargetPositionFromTargetY(DoubleSupplier targetYSup) { 
        return moveToTargetPosition(armPosLookupTableFromTargetY.get(targetYSup.getAsDouble()));
    }   

    private boolean armAtSetpoint() {
        return Math.abs(armMotor.relativeEncoder.getPosition() - armMotor.getSetpoint()) < armSetpointTolerance;
    }

    private boolean flywheelsAtSetpoint() {
        return Math.abs(flywheelMotor.relativeEncoder.getVelocity() - flywheelMotor.getSetpoint()) < flywheelVelocityTolerance;
    }

    private void reset() {
        armMotor.setClosedLoopTarget(armMotor.getAbsolutePosition());
        flywheelMotor.setClosedLoopTarget(0.0);
        passthroughStorageMotor.set(0.0);
    }

    public Command resetMotors() {
        return runOnce(this::reset);
    }
}
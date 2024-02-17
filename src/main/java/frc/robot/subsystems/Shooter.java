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

    private final SparkMaxWrapper flywheelMotor = null;//TODO add back flywheel code
    private final SparkMaxWrapper flywheelMotorFollower = null;

    private final SparkMaxWrapper passthroughStorageMotor = null;

    private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

    private final DigitalInput beamBreak = new DigitalInput(beamBreakPort);

    public Shooter() { 
        armMotor = new SparkMaxWrapper(shooterArmConstants);
        armMotor.configAbsoluteEncoder(invertAbsoluteEncoder, absoluteEncoderConversionFactor, absoluteEncoderOffset);
        armMotor.config();

        armMotorFollower = new SparkMaxWrapper(shooterArmFolllowerConstants);
        armMotorFollower.follow(armMotor, invertArmFollower);
        armMotorFollower.config();

        // flywheelMotor = new SparkMaxWrapper(shooterFlywheelConstants);
        // flywheelMotor.config();

        // flywheelMotorFollower = new SparkMaxWrapper(shooterFlywheelFolllowerConstants);
        // flywheelMotorFollower.follow(flywheelMotor, invertFlywheelFollower);
        // flywheelMotorFollower.config();

        // passthroughStorageMotor = new SparkMaxWrapper(shooterStorageConstants);
        // passthroughStorageMotor.config();

         /* Wait for the encoder to initialize before setting to absolute */
        Timer.delay(1.0);

         /* Reset the relative encoder to the absolute encoder value */
        armMotor.relativeEncoder.setPosition(armMotor.getAbsolutePosition());

        /* Add Telemetry */
        shooterTab.add(armMotor)
            .withPosition(0, 0).withSize(2, 2);
        // shooterTab.add(flywheelMotor)
            // .withPosition(3, 0).withSize(2, 2);
        shooterTab.addBoolean("Beam Break Triggered", this::isBeamBreakTriggered)
            .withPosition(6, 0).withSize(2, 1);

        /* Prevent moving to a previous setpoint */
        armMotor.setClosedLoopTarget(armMotor.getAbsolutePosition());
        // flywheelMotor.setClosedLoopTarget(0.0);
    }

    public boolean isBeamBreakTriggered() {
        return !beamBreak.get();//TODO true or false?
    }

    public Command enableStorageMotorReceiving() {
        return runOnce(() -> passthroughStorageMotor.set(storageMotorPowerReceiving));
    }

    private Command enableStorageMotorToFlywheels() {
        return runOnce(() -> passthroughStorageMotor.set(storageMotorPowerToFlywheels));
    }

    public Command disableStorageMotor() {
        return runOnce(() -> passthroughStorageMotor.set(0.0));
    }

    private Command waitForShooterVelocity(double velocity) {
        return new FunctionalCommand(
        /* Sets the target velocity at the start */
        () -> flywheelMotor.setClosedLoopTarget(velocity), 
        () -> {}, 
        (unused) -> {}, 
        /* We are finished if the flywheel velocity is within our tolerance */
        () -> Math.abs(flywheelMotor.relativeEncoder.getVelocity() - velocity) < flywheelVelocityTolerance, 
        this);
    }

    private Command disableShooterFlywheel() {
        return runOnce(() -> flywheelMotor.setClosedLoopTarget(0.0));
    }

    public Command setShooterVelocityThenWaitThenDisable(double velocity, double waitTime) {
        return waitForShooterVelocity(velocity)
           .andThen(enableStorageMotorToFlywheels())
           .andThen(Commands.waitSeconds(waitTime))
           .andThen(disableShooterFlywheel())
           .andThen(disableStorageMotor());
    }

    public Command setToPassthroughPosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(armPositionPassthrough)); 
    }

    public Command setToStoragePosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(armPositionStorage)); 
    }

    /* Sets the target and wait until it is achieved */
    private Command moveArmToTargetPosition(double position) { 
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> armMotor.setClosedLoopTarget(position),
        () -> {},
        (unused) -> {},
        /* We are finished if the arm position is within our tolerance */
        () -> Math.abs(armMotor.relativeEncoder.getPosition() - position) < armSetpointTolerance,
        this);
    }   

    public Command moveArmToPassthroughPosition() {
        return moveArmToTargetPosition(armPositionPassthrough);
    }

    public Command moveArmToPositionFromTargetY(DoubleSupplier targetYSup) { 
        /* Sets the target position to an interpolated value from the lookup table */
        return moveArmToTargetPosition(armPosLookupTableFromTargetY.get(targetYSup.getAsDouble()));
    }   
}
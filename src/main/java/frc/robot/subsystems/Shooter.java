package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final SparkMaxWrapper armMotor;    
    private final SparkMaxWrapper armMotorFollower;    

    private final SparkMaxWrapper flywheelMotor;
    private final SparkMaxWrapper flywheelMotorFollower;

    private final SparkMaxWrapper storageMotor;

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

        storageMotor = new SparkMaxWrapper(shooterStorageConstants);
        storageMotor.config();

         /* Wait for the encoder to initialize before setting to absolute */
        Timer.delay(1.0);

         /* Reset the relative encoder to the absolute encoder value */
        armMotor.relativeEncoder.setPosition(armMotor.getAbsolutePosition());

        /* Add Telemetry */
        shooterTab.add(armMotor)
            .withPosition(0, 0).withSize(2, 2);
        shooterTab.add(flywheelMotor)
            .withPosition(3, 0).withSize(2, 2);
        shooterTab.add(storageMotor)
            .withPosition(0, 2).withSize(2, 1);
        shooterTab.addBoolean("Has Note", this::hasNote)
            .withPosition(6, 0).withSize(2, 1);
        shooterTab.addBoolean("Arm At Setpoint", this::armAtSetpoint)
            .withPosition(6, 1).withSize(2, 1);
        shooterTab.addBoolean("Flywheels At Setpoint", this::flywheelsAtSetpoint)
            .withPosition(6, 2).withSize(2, 1);
        shooterTab.add(this)
            .withPosition(0, 4).withSize(2, 1);

        /* Add Command Testing Butons */
        shooterTab.add(enableStorageMotorReceiving())
            .withPosition(0, 3).withSize(1, 1);
        shooterTab.add(enableStorageMotorToFlywheels())
            .withPosition(1, 3).withSize(1, 1);
        shooterTab.add(shootSequence(4000.0))
            .withPosition(2, 3).withSize(1, 1);
        shooterTab.add(resetState())
            .withPosition(3, 3).withSize(1, 1);
        shooterTab.add(setToStoragePosition(true))
            .withPosition(4, 3).withSize(1, 1);
        shooterTab.add(setToPassthroughPosition(true))
            .withPosition(5, 3).withSize(1, 1);

        /* Prevent moving to a previous setpoint */
        reset();
    }

    public boolean hasNote() {
        return !beamBreak.get();
    }

    private Command setStorageMotorPower(double power) {
        return runOnce(() -> storageMotor.set(power));
    }

    /* Sets the target and if blocking, waits until the setpoint is achieved */
    private Command setFlywheelVelocity(double setpoint, boolean blocking) {
        Command setTargetCommand = runOnce(() -> flywheelMotor.setClosedLoopTarget(setpoint));

        return blocking ? setTargetCommand.andThen(Commands.waitUntil(this::flywheelsAtSetpoint)) : setTargetCommand;
    }

    /* Sets the target and if blocking, waits until the setpoint is achieved */
    private Command setTargetPosition(double setpoint, boolean blocking) {
        Command setTargetCommand = runOnce(() -> armMotor.setClosedLoopTarget(setpoint));

        return blocking ? setTargetCommand.andThen(Commands.waitUntil(this::armAtSetpoint)) : setTargetCommand;
    }

    private Command enableStorageMotorReceiving() {
        return setStorageMotorPower(storageMotorPowerReceiving).withName("Storage receive");
    }

    private Command enableStorageMotorToFlywheels() {
        return setStorageMotorPower(storageMotorPowerToFlywheels).withName("Storage flywheels");
    }

    private Command disableShooterFlywheels() {
        return runOnce(() -> { 
            flywheelMotor.setClosedLoopTarget(0.0);
            storageMotor.set(0.0);
        });
    }

    public Command receiveNoteFromIntake() {
        return enableStorageMotorReceiving()
            .andThen(Commands.waitUntil(this::hasNote))
            .andThen(disableShooterFlywheels());
    }

    public Command shootSequence(double velocity) {
        return setFlywheelVelocity(velocity, true)
           .andThen(enableStorageMotorToFlywheels())
           .andThen(Commands.waitSeconds(shotWaitTime))
           .andThen(disableShooterFlywheels())
           .andThen(setToStoragePosition(false))
           .withName("Shoot");
    }

    public Command setToStoragePosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionStorage, waitUntilAchieved).withName("To storage"); 
    }

    public Command setToPassthroughPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionStorage, waitUntilAchieved).withName("To storage"); 
    }

    public Command setToAutoPosition(double position, boolean waitUntilAchieved) {
        return setTargetPosition(position, waitUntilAchieved).withName("To auto pos");
    }

    /* Moves to the target position from a vision target y offset */
    public Command setToTargetPositionFromTargetY(DoubleSupplier targetYSup, boolean waitUntilAchieved) { 
        return setTargetPosition(armPosLookupTableFromTargetY.get(targetYSup.getAsDouble()), waitUntilAchieved).withName("Move to LLT");
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
        storageMotor.set(0.0);
        if(getCurrentCommand() != null) 
            getCurrentCommand().cancel();
    }

    private Command resetState() {
        return runOnce(this::reset).withName("Reset");
    }
}
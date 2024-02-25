package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final SparkMaxWrapper armMotor;    
    private final SparkMaxWrapper flywheelMotor;

    private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

    private final DigitalInput beamBreak = new DigitalInput(beamBreakPort);

    public Intake() {
        armMotor = new SparkMaxWrapper(intakeArmConstants);
        armMotor.configAbsoluteEncoder(invertAbsoluteEncoder, absoluteEncoderConversionFactor, absoluteEncoderOffset);
        armMotor.config();

        flywheelMotor = new SparkMaxWrapper(intakeFlywheelConstants);
        flywheelMotor.config();

        /* Wait for the encoder to initialize before setting to absolute */
        Timer.delay(1.0);

        /* Reset the relative encoder to the absolute encoder value */
        armMotor.relativeEncoder.setPosition(armMotor.getAbsolutePosition());

        /* Add Telemetry */
        intakeTab.add(armMotor)
            .withPosition(0, 0).withSize(2, 2);
        intakeTab.add(flywheelMotor)
            .withPosition(3, 0).withSize(2, 1);
        intakeTab.addBoolean("Has Note", this::hasNote)
            .withPosition(6, 0).withSize(2, 1);
        intakeTab.addBoolean("At Setpoint", this::armAtSetpoint)
            .withPosition(6, 1).withSize(2, 1);
        intakeTab.add(this)
            .withPosition(0, 3).withSize(2, 1);

        /* Add Command Testing Butons */
        intakeTab.add(setToAmpPosition())
            .withPosition(0, 2).withSize(1, 1);
        intakeTab.add(setToStoragePosition())
            .withPosition(1, 2).withSize(1, 1);
        intakeTab.add(moveToAmpPosition())
            .withPosition(2, 2).withSize(1, 1);
        intakeTab.add(moveToPassthroughPosition())
            .withPosition(3, 2).withSize(1, 1);
        intakeTab.add(shootIntoAmpThenWaitThenDisable())
            .withPosition(4, 2).withSize(1, 1);
        intakeTab.add(shootIntoShooter())
            .withPosition(5, 2).withSize(1, 1);
        intakeTab.add(setToGroundPositionAndEnable())
            .withPosition(6, 2).withSize(1, 1);
        intakeTab.add(resetMotors())
            .withPosition(7, 2).withSize(1, 1);
        
        /* Prevent moving to a previous setpoint */
        reset();
    }

    public boolean hasNote() {
        return !beamBreak.get();
    }

    /* Sets the target and disables (non-blocking) */
    private void setTargetPosition(double setpoint) {
        armMotor.setClosedLoopTarget(setpoint);
        flywheelMotor.set(0.0);
    }

    public Command setToAmpPosition() {
        return runOnce(() -> setTargetPosition(armPositionAmp));
    }

    public Command setToStoragePosition() {
        return runOnce(() -> setTargetPosition(armPositionStorage));
    }

    /* Sets the target and wait until it is achieved */
    private Command moveToTargetPosition(double position) {
        return new FunctionalCommand(
        /* Sets the target position and disable at the start */
        () -> setTargetPosition(position),
        () -> {},
        (unused) -> {},
         /* We are finished if the arm position is within our tolerance */
        this::armAtSetpoint,
        this);
    } 

    public Command moveToAmpPosition() {
        return moveToTargetPosition(armPositionAmp);
    }

    public Command moveToPassthroughPosition() {
        return moveToTargetPosition(armPositionPassthrough);
    }

    public Command shootIntoAmpThenWaitThenDisable() {
        return runOnce(() -> flywheelMotor.set(outtakeAmpPower))
            .andThen(Commands.waitSeconds(shotWaitTime))
            .andThen(runOnce(() -> flywheelMotor.set(0.0)));
    }

    public Command shootIntoShooter() {
        return runOnce(() -> flywheelMotor.set(outtakeToShooterPower));
    }

    public Command setToGroundPositionAndEnable() {
        return runOnce(() -> { 
            armMotor.setClosedLoopTarget(armPositionGround);
            flywheelMotor.set(intakePower);
        });
    }

    private boolean armAtSetpoint() {
        return Math.abs(armMotor.relativeEncoder.getPosition() - armMotor.getSetpoint()) < armSetpointTolerance;
    }

    private void reset() {
        armMotor.setClosedLoopTarget(armMotor.getAbsolutePosition());
        flywheelMotor.set(0.0);
        if(this.getCurrentCommand() != null) 
            this.getCurrentCommand().cancel();
    }

    public Command resetMotors() {
        return runOnce(this::reset);
    }
}
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
            .withPosition(3, 0).withSize(2, 2);
        intakeTab.addBoolean("Has Note", this::hasNote)
            .withPosition(6, 0).withSize(2, 1);;

        /* Prevent moving to a previous setpoint */
        armMotor.setClosedLoopTarget(armMotor.getAbsolutePosition());
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
        () -> Math.abs(armMotor.relativeEncoder.getPosition() - position) < armSetpointTolerance,
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
}
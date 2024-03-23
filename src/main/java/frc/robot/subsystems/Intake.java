package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final SparkMaxWrapper armMotor;    
    private final SparkMaxWrapper rollerMotor;

    private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

    private final DigitalInput beamBreak = new DigitalInput(beamBreakPort);

    private boolean hasNoteState = false;

    public Intake() {
        armMotor = new SparkMaxWrapper(intakeArmConstants);
        armMotor.config();

        rollerMotor = new SparkMaxWrapper(intakeRollerConstants);
        rollerMotor.config();

        armMotor.relativeEncoder.setPosition(armPositionStarting);

        /* Add Telemetry */
        intakeTab.add(armMotor)
            .withPosition(0, 0).withSize(2, 2);
        intakeTab.add(rollerMotor)
            .withPosition(3, 0).withSize(2, 1);
        intakeTab.addBoolean("Beam Break Triggered", this::beamBreakTriggered)
            .withPosition(6, 0).withSize(2, 1);
        intakeTab.addBoolean("Has Note State", () -> hasNoteState)
            .withPosition(6, 2).withSize(2, 1);
        intakeTab.addBoolean("At Setpoint", this::armAtSetpoint)
            .withPosition(6, 1).withSize(2, 1);
        intakeTab.add(this)
            .withPosition(0, 4).withSize(2, 1);

        /* Add Command Testing Butons */
        intakeTab.add(setToGroundPosition(true))
            .withPosition(0, 3).withSize(1, 1);
        intakeTab.add(setToPassthroughPosition(true))
            .withPosition(1, 3).withSize(1, 1);
        intakeTab.add(setToAmpPosition(true))
            .withPosition(2, 3).withSize(1, 1);
        intakeTab.add(shootIntoAmp())
            .withPosition(3, 3).withSize(1, 1);
        intakeTab.add(enableTransferToShooter())
            .withPosition(4, 3).withSize(1, 1);
        intakeTab.add(getNoteFromGround())
            .withPosition(5, 3).withSize(1, 1);
        intakeTab.add(resetCommand())
            .withPosition(6, 3).withSize(1, 1);
    }

    /* Sets the target and if blocking, waits until the setpoint is achieved */
    private Command setTargetPosition(double setpoint, boolean blocking) {
        Command setTargetCommand = runOnce(() -> armMotor.setClosedLoopTarget(setpoint));

        return blocking ? setTargetCommand.andThen(Commands.waitUntil(this::armAtSetpoint)) : setTargetCommand;
    }

    private Command setRollerPower(double power) {
        return runOnce(() -> rollerMotor.set(power));
    }

    private Command setNoteState(boolean hasNoteState) {
        return runOnce(() -> this.hasNoteState = hasNoteState);
    }

    public Command setToGroundPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionGround, waitUntilAchieved).withName("To ground");
    }
    
    public Command setToPassthroughPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionPassthrough, waitUntilAchieved).withName("To passthrough");
    }

    public Command setToAmpPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionAmp, waitUntilAchieved).withName("To amp");
    }

    public Command setToEjectPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionEject, waitUntilAchieved).withName("To eject");
    }

    public Command disableRollers() {
        return setRollerPower(0.0);
    }

    public Command shootIntoAmp() {
        return disableRollers()
            .andThen(setToAmpPosition(true))
            .andThen(Commands.waitSeconds(ampWaitTime))
            .andThen(setRollerPower(outtakeAmpPower))
            .andThen(Commands.waitSeconds(shotWaitTime))
            .andThen(disableRollers())
            .andThen(setNoteState(false))
            .andThen(setToPassthroughPosition(false))
            .withName("Shoot into amp");
    }

    public Command ejectNote() {
        return disableRollers()
            .andThen(setToEjectPosition(true))
            .andThen(setRollerPower(outtakeAmpPower))
            .andThen(Commands.waitSeconds(shotWaitTime))
            .andThen(disableRollers())
            .andThen(setNoteState(false))
            .andThen(setToPassthroughPosition(false))
            .withName("Eject");
    }

    public Command enableTransferToShooter() {
        return setRollerPower(transferToShooterPower)
            .andThen(setNoteState(false))
            .withName("Transfer");
    }

    public Command getNoteFromGround() {
        return setTargetPosition(armPositionGround, false)
            .andThen(setRollerPower(intakePower))
            .andThen(Commands.waitUntil(this::beamBreakTriggered))
            .andThen(disableRollers())
            .andThen(setNoteState(true))
            .andThen(Commands.print("Intake received note"))
            .andThen(setToPassthroughPosition(false))
            .withName("Get note");
    }

    public boolean hasNote() {
        return hasNoteState;
    }

    public boolean armAtSetpoint() {
        return Math.abs(armMotor.relativeEncoder.getPosition() - armMotor.getSetpoint()) < armSetpointTolerance;
    }

    private boolean beamBreakTriggered() {
        return !beamBreak.get();
    }

    public void reset() {
        if(this.getCurrentCommand() != null) 
            this.getCurrentCommand().cancel();

        rollerMotor.set(0.0);
        armMotor.setClosedLoopTarget(armMotor.relativeEncoder.getPosition());

        hasNoteState = false;
    }

    public Command resetCommand() {
        return runOnce(this::reset).withName("Reset");
    }
}
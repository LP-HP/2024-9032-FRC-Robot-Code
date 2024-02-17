package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
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
        armMotor.configAbsoluteEncoder(invertAbsoluteEncoder);
        armMotor.config();

        flywheelMotor = new SparkMaxWrapper(intakeFlywheelConstants);
        flywheelMotor.config();

        /* Wait for the encoder to initialize before setting to absolute */
        Timer.delay(1.0);

        /* Reset the relative encoder to the absolute encoder value */
        armMotor.relativeEncoder.setPosition(armMotor.getAbsolutePosition());

        /* Add Telemetry */
        intakeTab.add(armMotor)
            .withPosition(1, 1).withSize(2, 4);
        intakeTab.add(flywheelMotor)
            .withPosition(4, 1).withSize(2, 4);
        intakeTab.addBoolean("Beam Break Triggered", this::isBeamBreakTriggered)
            .withPosition(1, 4).withSize(2, 1);;

        /* Prevent moving to a previous setpoint */
        armMotor.setClosedLoopTarget(armMotor.relativeEncoder.getPosition());
    }

    public boolean isBeamBreakTriggered() {
        return !beamBreak.get();//TODO true or false?
    }

    /* Just sets the target */
    public Command setToAmpPosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(armPositionAmp));
    }

    public Command setToPassthroughPosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(armPositionPassthrough));
    }

    public Command setToStoragePosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(armPositionStorage));
    }

    /* Sets the target and wait until it is achieved */
    private Command moveToTargetPosition(double position) {
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> armMotor.setClosedLoopTarget(position),
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

    public Command disableIntake() {
        return runOnce(() -> flywheelMotor.set(0.0));
    }

    public Command shootIntoAmp() {
        return runOnce(() -> flywheelMotor.set(outtakeAmpPower));
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
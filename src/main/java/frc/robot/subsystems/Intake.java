package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SparkMaxWrapper;

public class Intake extends SubsystemBase {
    private SparkMaxWrapper armMotor;    
    private SparkMaxWrapper flywheelMotor;

    private final DigitalInput beamBreak = new DigitalInput(Constants.IntakeConstants.beamBreakPort);

    public Intake() {
        armMotor = new SparkMaxWrapper(Constants.IntakeConstants.intakeArmConstants);
        armMotor.configAbsoluteEncoder(Constants.IntakeConstants.invertAbsoluteEncoder);
        armMotor.config();

        flywheelMotor = new SparkMaxWrapper(Constants.IntakeConstants.intakeFlywheelConstants);
        flywheelMotor.config();

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

    /* Just sets the target */
    public Command setToGroundPosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(Constants.IntakeConstants.armPositionGround));
    }

    public Command setToAmpPosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(Constants.IntakeConstants.armPositionAmp));
    }

    public Command setToPassthroughPosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(Constants.IntakeConstants.armPositionPassthrough));
    }

    public Command setToStoragePosition() {
        return runOnce(() -> armMotor.setClosedLoopTarget(Constants.IntakeConstants.armPositionStorage));
    }

    /* Sets the target and wait until it is achieved */
    private Command moveToTargetPosition(double position) {
        return new FunctionalCommand(
        /* Sets the target position at the start */
        () -> armMotor.setClosedLoopTarget(position),
        () -> {},
        (unused) -> {},
         /* We are finished if the arm position is within our tolerance */
        () -> Math.abs(armMotor.relativeEncoder.getPosition() - position) < Constants.IntakeConstants.armSetpointTolerance,
        this);
    } 

    public Command moveToAmpPosition() {
        return moveToTargetPosition(Constants.IntakeConstants.armPositionAmp);
    }

    public Command moveToPassthroughPosition() {
        return moveToTargetPosition(Constants.IntakeConstants.armPositionPassthrough);
    }
    
    public Command enableIntake() {
        return runOnce(() -> flywheelMotor.set(Constants.IntakeConstants.intakePower));
    }

    public Command disableIntake() {
        return runOnce(() -> flywheelMotor.set(0));
    }

    public Command shootIntoAmp() {
        return runOnce(() -> flywheelMotor.set(Constants.IntakeConstants.outtakeAmpPower));
    }

    public Command shootIntoShooter() {
        return runOnce(() -> flywheelMotor.set(Constants.IntakeConstants.outtakeToShooterPower));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break Triggered", isBeamBreakTriggered());
    }
}
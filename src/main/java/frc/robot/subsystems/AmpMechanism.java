package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AmpMechanismConstants.*;

import java.util.function.DoubleSupplier;

public class AmpMechanism extends SubsystemBase {
    private final Servo ampServoLeft;
    private final Servo ampServoRight;

    public AmpMechanism() {
        ampServoLeft = new Servo(leftServoPort);
        ampServoRight = new Servo(rightServoPort);

        var tab = Shuffleboard.getTab("Debug");

        GenericEntry servoSetpoint = tab.add("Amp Mech Setpoint", downPosition)
            .getEntry();

        tab.add(setPosWithSupplier(() -> servoSetpoint.getDouble(downPosition)).withName("Override Amp Mech"));
    }

    private Command setPosWithSupplier(DoubleSupplier posSup) {
        return runOnce(() -> setPos(posSup.getAsDouble()));
    }

    private Command setPosFromConstant(double position) {
        return runOnce(() -> setPos(position));
    }

    private void setPos(double position) {
        ampServoLeft.set(position);
        ampServoRight.set(1.0 - position);
    }

    public Command prepare() {
        return setPosFromConstant(upPosition)
            .andThen(Commands.waitSeconds(ampPrepareWait));
    }

    public Command guideNote() {
        return setPosFromConstant(scorePosition)
            .andThen(Commands.waitSeconds(ampScoreWait))
            .andThen(setPosFromConstant(downPosition));
    }
}

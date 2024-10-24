package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AmpMechanismConstants.*;

public class AmpMechanism extends SubsystemBase {
    private final Servo ampServoLeft;
    private final Servo ampServoRight;

    public AmpMechanism() {
        ampServoLeft = new Servo(leftServoPort);
        ampServoRight = new Servo(rightServoPort);

        var tab = Shuffleboard.getTab("Debug");
        tab.add(ampServoLeft);
        tab.add(ampServoRight);
    }

    private Command setPos(double position) {
        return runOnce(() -> {
            ampServoLeft.set(position);
            ampServoRight.set(Math.abs(1.0 - position));
        });
    }

    public Command guideNote() {
        return setPos(upPosition)
            .andThen(Commands.waitSeconds(ampScoreWait))
            .andThen(setPos(downPosition));
    }
}

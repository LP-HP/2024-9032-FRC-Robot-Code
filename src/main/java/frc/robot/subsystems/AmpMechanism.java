package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AmpMechanismConstants.*;

public class AmpMechanism extends SubsystemBase {
    private final Servo ampServo;

    public AmpMechanism() {
        ampServo = new Servo(servoPort);
    }

    private Command setPos(double position) {
        return runOnce(() -> ampServo.set(position));
    }

    public Command guideNote() {
        return setPos(upPosition)
            .andThen(Commands.waitSeconds(ampScoreWait))
            .andThen(setPos(downPosition));
    }
}

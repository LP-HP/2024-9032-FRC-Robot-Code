package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpMechanismConstants;

public class AmpMechanism extends SubsystemBase {
    Servo ampServo;

    public AmpMechanism() {
        ampServo = new Servo(AmpMechanismConstants.ampServoPort);
    }

    public void setPos(double position) {
        ampServo.set(position);
    }

    public Command move() {
        return runOnce(() -> setPos(AmpMechanismConstants.ampMechanismUp));
    }
}

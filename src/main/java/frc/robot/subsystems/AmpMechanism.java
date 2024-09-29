package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpMechanism extends SubsystemBase {
    private int servoLeftPort;
    private int servoRightPort;

    Servo servoLeft;

    public AmpMechanism(int servo1, int servo2) {
        servoLeftPort = servo1;
        servoRightPort = servo2;
        //Servo
        servoLeft = new Servo(servoLeftPort);
    }

    public void setPosAngular(double angle) {
        servoLeft.setAngle(angle);
    }

    public void setPos(double position) {
        servoLeft.set(position);
    }

    public Command setPositionAngular(double angle) {
        return runOnce(() -> setPosAngular(angle));
    }

    public Command setPosition(double position) {
        return runOnce(() -> setPosAngular(position));
    }
}

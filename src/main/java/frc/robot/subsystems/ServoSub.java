import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSub extends SubsystemBase{
    Servo servo;
    public ServoSub(int port){
        servo = new Servo(port);

    }

    public Command setPos(int position){
        return runOnce(() -> servo.set(position));
    }
    

}
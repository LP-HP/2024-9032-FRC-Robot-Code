package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSensor extends SubsystemBase {
    private final Ultrasonic sensor;
    private final ShuffleboardTab UltrasonicTab = Shuffleboard.getTab("UltrasonicTab");
    private Double sensorDistance;

    public UltrasonicSensor(int ping, int echo) {
        sensor = new Ultrasonic(ping, echo);
        sensor.setAutomaticMode(true);
        UltrasonicTab.addDouble("Distance", () -> sensorDistance)
                .withPosition(1, 1).withSize(2, 1);
        UltrasonicTab.add(toggle())
                .withPosition(1, 1).withSize(2, 1);
    }

    @Override
    public void periodic() {
        sensorDistance = sensor.getRangeMM();
        System.out.println(sensorDistance);
    }

    public void toggleSensor() {
        sensor.setAutomaticMode(sensor.isEnabled() ? true : false);
    }

    public double getDistanceInMM() {
        return sensorDistance;
    }

    public Command toggle() {
        return runOnce(this::toggleSensor).withName("Toggle");
    }
}

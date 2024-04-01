package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSensor extends SubsystemBase {
    private final Ultrasonic sensor;
    private final ShuffleboardTab UltrasonicTab = Shuffleboard.getTab("UltrasonicTab");
    private Double sensorDistance;
    private boolean autoSensorPinging = false;

    public UltrasonicSensor(int ping, int echo) {
        sensor = new Ultrasonic(ping, echo);
        Ultrasonic.setAutomaticMode(true);
        UltrasonicTab.addDouble("Distance", () -> sensorDistance)
                .withPosition(1, 1).withSize(2, 1);
    }

    @Override
    public void periodic() {
        sensorDistance = sensor.getRangeMM();
    }

    public void toggleSensorPing() {
        Ultrasonic.setAutomaticMode(autoSensorPinging == true ? false : true);
    }

    public double pingOnceAndRecieve() {
        sensor.ping();
        return sensor.getRangeMM();
    }

    public double getDistanceInMM() {
        return sensorDistance;
    }
}

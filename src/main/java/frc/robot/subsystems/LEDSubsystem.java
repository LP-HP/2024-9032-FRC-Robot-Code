package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    public static enum LEDState { RAINBOW };
    private LEDState currentState;

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private int previousFirstLEDHue;

    public LEDSubsystem() {
        ledStrip = new AddressableLED(ledPWMPort);
        ledBuffer = new AddressableLEDBuffer(ledStripLength);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case RAINBOW:
                rainbow();
                break;
            default:
                break;
        }
    }

    private void rainbow() {
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            int hue = (previousFirstLEDHue + (i * 180 / ledBuffer.getLength())) % 180;

            ledBuffer.setHSV(i, hue, 255, 128);
        }

        /* Increment the starting color to create a smooth rainbow and prevent overflowing */
        previousFirstLEDHue += 3;
        previousFirstLEDHue %= 180;

        ledStrip.setData(ledBuffer);
    }
}
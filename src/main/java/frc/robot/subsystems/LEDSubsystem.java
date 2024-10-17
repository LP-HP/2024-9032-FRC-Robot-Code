package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    public static enum LEDState { RAINBOW, FAST_BLUE_GRADIENT, GREEN_GRADIENT, ORANGE_GRADIENT, SLOW_BLUE_GRADIENT };
    private LEDState currentState;

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private int previousFirstLEDHue;
    private int previousFirstLEDValue;

    public LEDSubsystem(LEDState startingState) {
        ledStrip = new AddressableLED(ledPWMPort);
        ledBuffer = new AddressableLEDBuffer(ledStripLength);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        currentState = startingState;
    }

    public Command setState(LEDState state) {
        return runOnce(() -> {
            currentState = state;

            resetLEDs();
        });
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case RAINBOW:
                rainbow();
                break;
            case FAST_BLUE_GRADIENT:
                gradient(119, 5);
                break;
            case GREEN_GRADIENT:
                gradient(238, 5);
                break;
            case SLOW_BLUE_GRADIENT:
                gradient(238, 1);
                break;
            case ORANGE_GRADIENT:
                gradient(24, 10);
            default:
                break;
        }
    }

    private void resetLEDs() {
        for(int i = 0; i < ledBuffer.getLength(); i++) 
            ledBuffer.setHSV(i, 0, 0, 0);

        previousFirstLEDHue = 0;
        previousFirstLEDValue = 0;
    }

    private void rainbow() {
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            int hue = (previousFirstLEDHue + (i * 180 / ledBuffer.getLength())) % 180;

            ledBuffer.setHSV(i, hue, 255, 128);
        }

        /* Increment the previous color to create a smooth rainbow and prevent overflowing */
        previousFirstLEDHue += 3;
        previousFirstLEDHue %= 180;

        ledStrip.setData(ledBuffer);
    }

    private void gradient(int hue, int increment) {
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            int value = (previousFirstLEDValue + (i * 255 / ledBuffer.getLength())) % 255; 

            ledBuffer.setHSV(i, hue, 255, value);
        }

        /* Increment the previous value to create a smooth gradient and prevent overflowing */
        previousFirstLEDValue += increment;
        previousFirstLEDValue %= 255;

        ledStrip.setData(ledBuffer);
    }
}
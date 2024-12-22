package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    public static enum LEDState { RAINBOW, FAST_BLUE_GRADIENT, GREEN_GRADIENT, ORANGE_GRADIENT, SLOW_BLUE_GRADIENT, RED_GRADIENT };
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
        })
        .ignoringDisable(true);
    }
    int redGreen = 0;
    int inc = 0;
    @Override
    public void periodic() {
        switch (currentState) {
            case RAINBOW:
            case FAST_BLUE_GRADIENT:
            case GREEN_GRADIENT:
            case SLOW_BLUE_GRADIENT:
            case ORANGE_GRADIENT:
            case RED_GRADIENT:
            default: 
            inc++;
            if(inc%10 == 0){
                inc %= 10;
                if(redGreen == 0){redGreen = 1;} else{redGreen = 0;}
                for(int i = 0; i < ledBuffer.getLength(); i++) {
                    if(i%2==redGreen){
                        ledBuffer.setRGB(i,255,0,0);
                    } else{
                        ledBuffer.setRGB(i,0,255,0);
                    }
                }
            } 
            
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
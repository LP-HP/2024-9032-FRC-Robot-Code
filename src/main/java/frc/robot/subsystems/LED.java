package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    int m_rainbowFirstPixelHue;
    int r;
    //private final ShuffleboardTab LEDtab = Shuffleboard.getTab("LED tab");

    public LED(int PWMPort, int LEDLength) {
        led = new AddressableLED(PWMPort);
        ledBuffer = new AddressableLEDBuffer(LEDLength);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
        setLedColor(0, 0, 139);
    }
    /*
    @Override
    public void periodic() {
        //rainbow();
        //led.setData(ledBuffer);
    }
    */

    public Runnable setLedColor(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, r, g, b);
         }
         
         led.setData(ledBuffer);
        return null;
    }
    private void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }
    public Command setColor(int r, int g, int b) {
        return runOnce(setLedColor(r, b, g)).withName("set LED color");
    }
    public Runnable shootingLEDPattern() throws InterruptedException {
        for (var i = 0; i< ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 95, 31);
            Thread.sleep(25);
            led.setData(ledBuffer);
        }
        return null;
    }
    public Command shootingLED() throws InterruptedException {
        return runOnce(shootingLEDPattern());
    }
}

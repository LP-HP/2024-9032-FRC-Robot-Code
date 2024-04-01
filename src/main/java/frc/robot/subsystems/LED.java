package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
      }
    public Command setColor(int r, int g, int b) {
        return runOnce(setLedColor(r, b, g)).withName("set LED color");
    }
    public Runnable shootingLEDPattern() {
        for (var i = 0; i< ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 95, 31);
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                System.out.println("LED's interrupted");
            }
            led.setData(ledBuffer);
            ledBuffer.setRGB(i, 0, 0, 0);
            led.setData(ledBuffer);
        }
        return null;
    }
    public Command shootingLED() {
        return runOnce(shootingLEDPattern());
    }
}

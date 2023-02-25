package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedsConstants {
    static final AddressableLED LED = new AddressableLED(8);
    static final int LEDS_LENGTH = 300;
    static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(LEDS_LENGTH);

    static {
        LedsConstants.LED.setLength(LEDS_LENGTH);
        LedsConstants.LED.setData(LED_BUFFER);
        LedsConstants.LED.start();
    }
}
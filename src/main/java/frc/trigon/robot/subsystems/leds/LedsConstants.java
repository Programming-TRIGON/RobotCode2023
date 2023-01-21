package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedsConstants {
    static final AddressableLED LED = new AddressableLED(9);
    static final int LEDS_LENGTH = 20;
    static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(LEDS_LENGTH);

    static {
        LED.setLength(LED_BUFFER.getLength());
        LED.setData(LED_BUFFER);
        LED.start();
    }


}
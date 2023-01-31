package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.Robot;

import java.lang.reflect.Array;

public class Leds extends SubsystemBase {
    private final static Leds INSTANCE = new Leds();
    private final AddressableLED leds = LedsConstants.LED;
    private final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(LedsConstants.LEDS_LENGTH);


    private Leds() {
        LedsConstants.LED.setLength(LED_BUFFER.getLength());
        LedsConstants.LED.setData(LED_BUFFER);
        LedsConstants.LED.start();
    }

    public static Leds getInstance() {
        return INSTANCE;
    }

    private Color rgbToGrb(Color color) {
    if (Robot.isReal())
        return new Color(color.green, color.red, color.blue);
    else{return color;}
    }

    private Color brightness(Color color, double brightness){
        return new Color(color.red * brightness , color.green * brightness , color.blue * brightness);
    }

    public void setLedsColors(Color[] ledBuffer){
        for (int i = 0; i < ledBuffer.length; i++){
            LED_BUFFER.setLED(i,rgbToGrb(brightness(ledBuffer[i], 0.1)));
        }
        leds.setData(LED_BUFFER);
    }
}
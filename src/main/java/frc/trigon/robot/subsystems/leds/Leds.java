package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.color.*;
public class Leds extends SubsystemBase {
    private final static Leds INSTANCE = new Leds();
    private final AddressableLED leds = LedsConstants.LED;

    private Leds() {
    }

    public static Leds getInstance() {
        return INSTANCE;
    }

    public Command StaticColorCommand(Color color) {
        return new StartEndCommand(() -> turnColor(rgbToGrb(color)), () -> {
        }, this).ignoringDisable(true);
    }


    public Command CommandRainbow() {
        return new StartEndCommand(() -> rainbow(), () -> {
        }, this).ignoringDisable(true);
    }


    private void turnColor(Color color) {
        for (int i = 0; i < LedsConstants.LED_BUFFER.getLength(); i++) {
            LedsConstants.LED_BUFFER.setLED(i, color);
        }

        leds.setData(LedsConstants.LED_BUFFER);
    }


    public void rainbow() {
        //int hue = 50;
        for (int i = 0; i < LedsConstants.LED_BUFFER.getLength(); i++) {
            final int hue = (i * 180 / LedsConstants.LED_BUFFER.getLength()) % 180;
            LedsConstants.LED_BUFFER.setHSV(i, hue, 255, 128);
        }
        leds.setData(LedsConstants.LED_BUFFER);
    }


    private Color rgbToGrb(Color color) {
        return new Color(color.green, color.red, color.blue);

    }
    //private Color hsvToGrb(Color color) {
      //  color.
    //}
}
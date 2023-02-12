package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.logging.Logger;

public class LedStrip extends SubsystemBase {
    private int startingPosition, endingPosition;
    private final boolean inverted;
    private static final AddressableLED leds = LedsConstants.LED;
    private static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(LedsConstants.LEDS_LENGTH);

    static {
        LedsConstants.LED.setLength(LED_BUFFER.getLength());
        LedsConstants.LED.setData(LED_BUFFER);
        LedsConstants.LED.start();
    }

    /**
     * Constructs a new LedStrip.
     * LedStrip represents a subsection of the one long LED strip on the robot.
     *
     * @param startingPosition the first LED in the strip
     * @param endingPosition   the last LED in the strip
     * @param inverted
     */
    public LedStrip(int startingPosition, int endingPosition, boolean inverted) {
        this.startingPosition = startingPosition;
        this.endingPosition = endingPosition;
        this.inverted = inverted;
    }

    public void setLedsColors(Color[] colors) {
        if (colors.length != getLength()) {
            Logger.getGlobal().warning("frc.trigon.robot.subsystems.leds.LedStrip.setLedsColors(): Tried to apply a array that not in the correct size");
            return;
        }
        if (inverted){
            colors = InvertArray( colors);
        }
        for (int i = startingPosition; i < endingPosition + 1; i++) {
            LED_BUFFER.setLED(i, convertToGrbIfIsReal(applyBrightness(colors[i - startingPosition], 1)));
        }
        leds.setData(LED_BUFFER);
    }

    public int getLength() {
        return endingPosition - startingPosition + 1;
    }

    private static Color applyBrightness(Color color, double brightness) {
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    private static Color rgbToGrb(Color color) {
        return new Color(color.green, color.red, color.blue);
    }

    private static Color convertToGrbIfIsReal(Color color) {
        if (Robot.isReal()) {
            return rgbToGrb(color);
        } else return color;
    }

    private static Color[] InvertArray(Color[] colors) {
        Collections.reverse(Arrays.asList(colors));

        return colors;
    }
}

package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;

import java.util.HashMap;
import java.util.logging.Logger;

public class LedStrip extends SubsystemBase {
    private final boolean inverted;
    private final int length;
    private final AddressableLED led = LedsConstants.LED;
    private static final HashMap<Color, Color> COLOR_COLOR_HASH_MAP = new HashMap<>();

    /**
     * Constructs a new LedStrip
     *
     * @param length           length of the strip
     * @param inverted         whether the strip is inverted
     */
    public LedStrip(int length, boolean inverted) {
        this.length = length;
        this.inverted = inverted;
    }

    /**
     * @return the length of the strip.
     */
    public int getLength() {
        return length;
    }

    /**
     * Sets the color of the LEDs in the strip.
     */
    void setLedColors(Color[] colors) {
        if (colors.length != getLength()) {
            Logger.getGlobal().warning("frc.trigon.robot.subsystems.leds.LedStrip.setLedsColors(): Tried to apply an array with incorrect size");
            if (getCurrentCommand() != null)
                getCurrentCommand().cancel();
            return;
        }
        for (int i = 0; i < length; i++) {
            int invertedIndex = inverted ? length - i : i;
            LedsConstants.LED_BUFFER.setLED(i, convertToTrihardColorIfReal(applyBrightness(colors[invertedIndex], 1)));
        }
        led.setData(LedsConstants.LED_BUFFER);
    }

    private static Color applyBrightness(Color color, double brightness) {
        if(brightness == 1)
            return color;
        if(brightness == 0)
            return Color.kBlack;
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    public static Color rgbToGrb(Color color) {
        return new Color(color.green, color.red, color.blue);
    }

    public static Color balance(Color color) {
        double v = Math.max(Math.max(color.red, color.green), color.blue);
        Color newColor = new Color(color.red, color.green / 2, color.blue / 4);
        double newV = Math.max(Math.max(newColor.red, newColor.green), newColor.blue);
        double ratio = v / newV;
        return new Color(newColor.red * ratio, newColor.green * ratio, newColor.blue * ratio);
    }

    private Color convertToTrihardColorIfReal(Color color) {
        if (Robot.isReal()) {
            var saved = COLOR_COLOR_HASH_MAP.get(color);
            if(saved != null)
                return saved;
            var converted = rgbToGrb(balance(color));
            COLOR_COLOR_HASH_MAP.put(color, converted);
            return converted;
        }
        else return color;
    }
}

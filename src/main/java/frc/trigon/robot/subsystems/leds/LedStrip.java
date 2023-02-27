package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;

import java.util.logging.Logger;

public class LedStrip extends SubsystemBase {
    private final MasterLed masterLed = MasterLed.getInstance();
    private final int startingPosition, endingPosition;
    private final boolean inverted;
    private final AddressableLED led = LedsConstants.LED;
    private final int virtualLength;

    /**
     * Constructs a new LedStrip
     *
     * @param startingPosition the first LED of the strip
     * @param length           length of the strip
     * @param inverted         whether the strip is inverted
     * @param virtualLength    the length of a virtual strip that is not connected to the robot.
     */
    public LedStrip(int startingPosition, int length, boolean inverted, int virtualLength) {
        this.startingPosition = startingPosition;
        this.endingPosition = getEndingPosition(length);
        this.inverted = inverted;
        this.virtualLength = virtualLength;
        MasterLed.LED_STRIPS.add(this);
    }

    /**
     * Constructs a new LedStrip.
     *
     * @param startingPosition the first LED in the strip
     * @param length           length of the strip
     * @param inverted         whether the strip is inverted
     */
    public LedStrip(int startingPosition, int length, boolean inverted) {
        this(startingPosition, length, inverted, length);
    }

    /**
     * @return the length of the strip.
     */
    public int getLength() {
        return virtualLength;
    }

    /**
     * Sets the color of the LEDs in the strip.
     */
    void setLedColors(Color[] colors) {
        if (colors.length != getLength() || getLength() < endingPosition - startingPosition + 1) {
            Logger.getGlobal().warning("frc.trigon.robot.subsystems.leds.LedStrip.setLedsColors(): Tried to apply an array with incorrect size");
            if (getCurrentCommand() != null)
                getCurrentCommand().cancel();
            return;
        }
        for (int i = startingPosition; i < endingPosition + 1; i++) {
            int invertedIndex = inverted ? endingPosition + startingPosition - i : i;
            LedsConstants.LED_BUFFER.setLED(i, convertToTrihardColorIfReal(applyBrightness(colors[invertedIndex - startingPosition], 1)));
        }
        led.setData(LedsConstants.LED_BUFFER);
    }

    /**
     * cancels all commands that are overlapping with this strip.
     */
    void cancelOverlapping() {
        for (LedStrip ledStrip : MasterLed.LED_STRIPS) {
            if (ledStrip == this)
                continue;

            if (isOverlapping(ledStrip)) {
                Command cmd = ledStrip.getCurrentCommand();
                if (cmd != null)
                    cmd.cancel();
            }
        }
    }

    private int getEndingPosition(int length) {
        return length + startingPosition - 1;
    }

    private static Color applyBrightness(Color color, double brightness) {
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    private static Color rgbToGrb(Color color) {
        return new Color(color.green, color.red, color.blue);
    }

    private static Color balance(Color color) {
        double v = Math.max(Math.max(color.red, color.green), color.blue);
        Color newColor = new Color(color.red, color.green / 2, color.blue / 4);
        double newV = Math.max(Math.max(newColor.red, newColor.green), newColor.blue);
        double ratio = v / newV;
        return new Color(newColor.red * ratio, newColor.green * ratio, newColor.blue * ratio);
    }

    private Color convertToTrihardColorIfReal(Color color) {
        if (Robot.isReal())
            return rgbToGrb(balance(color));
        else return color;
    }

    private boolean isOverlapping(LedStrip otherLedStrip) {
        return (otherLedStrip.startingPosition >= this.startingPosition && otherLedStrip.startingPosition <= this.endingPosition)
                || (otherLedStrip.endingPosition >= this.startingPosition && otherLedStrip.endingPosition <= this.endingPosition);
    }
}

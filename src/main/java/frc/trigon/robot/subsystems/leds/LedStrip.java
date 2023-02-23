package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;

import java.util.ArrayList;
import java.util.logging.Logger;

public class LedStrip extends SubsystemBase {
    private final int startingPosition, endingPosition;
    private final boolean inverted;
    private final AddressableLED led = LedsConstants.LED;
    public static final ArrayList<LedStrip> LED_STRIPS = new ArrayList<>();
    private final int virtualLength;

    /**
     * @param startingPosition the first LED of the strip
     * @param endingPosition   the last LED of the strip
     * @param inverted         whether the strip is inverted
     * @param virtualLength    the length of a virtual strip that is not connected to the robot.
     */
    public LedStrip(int startingPosition, int endingPosition, boolean inverted, int virtualLength) {
        this.startingPosition = startingPosition;
        this.endingPosition = endingPosition;
        this.inverted = inverted;
        this.virtualLength = virtualLength;
        LED_STRIPS.add(this);
    }

    /**
     * @param startingPosition the first LED in the strip
     * @param endingPosition   the last LED in the strip
     * @param inverted         whether the strip is inverted.
     */
    public LedStrip(int startingPosition, int endingPosition, boolean inverted) {
        this(startingPosition, endingPosition, inverted, endingPosition - startingPosition + 1);
    }

    /**
     * sets the color of the LEDs in the strip.
     */
    void setLedsColors(Color[] colors) {
        if (colors.length != getLength() || getLength() < endingPosition - startingPosition + 1) {
            Logger.getGlobal().warning("frc.trigon.robot.subsystems.leds.LedStrip.setLedsColors(): Tried to apply a array that not in the correct size");
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
     * @return the length of the strip.
     */
    public int getLength() {
        return virtualLength;
    }

    private static Color applyBrightness(Color color, double brightness) {
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    private static Color rgbToGrb(Color color) {
        return new Color(color.green, color.red, color.blue);
    }

    private static Color balance(Color color) {
        double v = Math.max(Math.max(color.red, color.green), color.blue);
        var nc = new Color(color.red, color.green / 2, color.blue / 4);
        double newV = Math.max(Math.max(nc.red, nc.green), nc.blue);
        double ratio = v / newV;
        return new Color(nc.red * ratio, nc.green * ratio, nc.blue * ratio);
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

    /**
     * cancels all commands that are overlapping with this strip.
     */
    void cancelOverlapping() {
        for (LedStrip ledStrip : LED_STRIPS) {
            if (ledStrip == this)
                continue;

            if (isOverlapping(ledStrip)) {
                Command cmd = ledStrip.getCurrentCommand();
                if (cmd != null)
                    cmd.cancel();
            }
        }
    }
}

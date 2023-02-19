package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.logging.Logger;

public class LedStrip extends SubsystemBase {
    private final int startingPosition, endingPosition;
    private final boolean invert;
    private static final AddressableLED leds = LedsConstants.LED;
    private static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(LedsConstants.LEDS_LENGTH);
    public static final ArrayList<LedStrip> ledStrips = new ArrayList<>();
    private final int virtualLength;

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
     * @param invert           whether the strip is inverted.
     * @param virtualLength    the length of a virtual strip that is not connected to the robot.
     */
    public LedStrip(int startingPosition, int endingPosition, boolean invert, int virtualLength) {
        this.startingPosition = startingPosition;
        this.endingPosition = endingPosition;
        this.invert = invert;
        this.virtualLength = virtualLength;
        ledStrips.add(this);
    }


    /**
     * Constructs a new LedStrip.
     * LedStrip represents a subsection of the one long LED strip on the robot.
     *
     * @param startingPosition the first LED in the strip
     * @param endingPosition   the last LED in the strip
     * @param invert whether the strip is inverted.
     */
    public LedStrip(int startingPosition, int endingPosition, boolean invert) {
        this(startingPosition, endingPosition, invert, endingPosition - startingPosition + 1);
    }

    public void setLedsColors(Color[] colors) {
        if (colors.length != getLength() || getLength() < endingPosition - startingPosition + 1) {
            Logger.getGlobal().warning("frc.trigon.robot.subsystems.leds.LedStrip.setLedsColors(): Tried to apply a array that not in the correct size");
            if (getCurrentCommand() != null)
                getCurrentCommand().cancel();
            return;
        }
        for (int i = startingPosition; i < endingPosition + 1; i++) {
            int idx = invert ? endingPosition + startingPosition - i : i;
            LED_BUFFER.setLED(i, convertToTrihardColorIfIsReal(applyBrightness(colors[idx - startingPosition], 1)));
        }
        leds.setData(LED_BUFFER);
    }

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


    private static Color convertToTrihardColorIfIsReal(Color color) {
        if (Robot.isReal()) {
            return rgbToGrb(balance(color));
        } else return color;
    }

    private static void InvertArray(Color[] colors) {
        Collections.reverse(Arrays.asList(colors));
    }

    boolean isOverlapping(LedStrip otherLedStrip) {
        return (otherLedStrip.startingPosition >= this.startingPosition && otherLedStrip.startingPosition <= this.endingPosition)
                || (otherLedStrip.endingPosition >= this.startingPosition && otherLedStrip.endingPosition <= this.endingPosition);
    }

    void cancelOverlapping() {
        for (LedStrip ledStrip : ledStrips) {
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

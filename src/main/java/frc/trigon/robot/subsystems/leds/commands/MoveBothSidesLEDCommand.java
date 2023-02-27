package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.leds.MasterLed;

public class MoveBothSidesLEDCommand extends LedCommand {
    private final Color backgroundColor;
    private final Color primeColor;
    private final double cycleTime;
    private final int amountOfMovingLeds;
    private final LedStrip ledStrip;

    /**
     * Constructs a new MoveBothSidesLEDCommand.
     *
     * @param backgroundColor    the color of the background
     * @param primeColor         the color of the moving leds
     * @param cycleTime          the time it takes for the leds to move from one pixel to the other
     * @param amountOfMovingLeds the amount of leds that move
     * @param ledStrip           led strip
     */
    public MoveBothSidesLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip) {
        super(ledStrip);
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds - 1;
        this.ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        int firstInMovingRange = getFirstInMovingRange(ledStrip.getLength());
        int lastInMovingRange = (firstInMovingRange + amountOfMovingLeds) % ledStrip.getLength();
        DefineTheArrayOfTheColors(colors, firstInMovingRange, lastInMovingRange);
        setLeds(colors);
    }

    private int getFirstInMovingRange() {
        return (int) (Timer.getFPGATimestamp() / cycleTime) ;
    }

    private int getFirstInMovingRange(int lengthOfStrip) {
        return getFirstInMovingRange() % lengthOfStrip;
    }

    private boolean shouldBePrimeColor(int index, int firstInMovingRange, int lastInMovingRange) {
        return isPositionInLedInRange(firstInMovingRange, lastInMovingRange, index) ||
                isSplitByEnd(firstInMovingRange, lastInMovingRange)||
                isInvertedSplitByEnd(lastInMovingRange, index)||
                isInvertedPositionInLedInRange(firstInMovingRange, lastInMovingRange, index);
    }

    private boolean isPositionInLedInRange(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return positionInLED >= firstInMovingRange && positionInLED <= lastInMovingRange;
    }

    private boolean isSplitByEnd(int lastInMovingRange, int positionInLED) {
        return positionInLED >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                positionInLED <= lastInMovingRange % ledStrip.getLength();
    }

    private boolean isInvertedPositionInLedInRange( int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return ((ledStrip.getLength() - positionInLED) >= firstInMovingRange && (ledStrip.getLength() - positionInLED) <= lastInMovingRange);
    }

    private boolean isInvertedSplitByEnd(int lastInMovingRange, int positionInLED) {
        return ((ledStrip.getLength() - positionInLED) >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                (ledStrip.getLength() - positionInLED) <= lastInMovingRange % ledStrip.getLength());
    }

    private void DefineTheArrayOfTheColors(Color[] colors, int firstInMovingRange, int lastInMovingRange) {
        for (int i = 0; i < ledStrip.getLength(); i++) {
            if (shouldBePrimeColor(i, firstInMovingRange, lastInMovingRange)) {
                colors[(ledStrip.getLength() - i) % ledStrip.getLength()] = primeColor;
                colors[i] = primeColor;
            } else {
                colors[(ledStrip.getLength() - i) % ledStrip.getLength()] = backgroundColor;
                colors[i] = backgroundColor;
            }
        }
    }
}
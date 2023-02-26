package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

public class MovingColorsLEDCommand extends LedCommand {
    private final Color backgroundColor;
    private final Color primeColor;
    private final double cycleTime;
    private final int amountOfMovingLeds;
    private final LedStrip ledStrip;

    /**
     * Construct a new MovingColorsLEDCommand.
     *
     * @param backgroundColor    The color of the background
     * @param primeColor         The color of the moving leds
     * @param cycleTime          The time it takes for the moving leds to go from one pixel to the other
     * @param amountOfMovingLeds The amount of leds that are moving
     * @param ledStrip           The led strip
     */
    public MovingColorsLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip) {
        super(ledStrip);
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds - 1;
        this.ledStrip = ledStrip;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        int firstInMovingRange = getFirstInMovingRange(ledStrip.getLength());
        int lastInMovingRange = firstInMovingRange + amountOfMovingLeds;
        defineTheArrayOfTheColors(colors, firstInMovingRange, lastInMovingRange);
        setLeds(colors);
    }

    private int getFirstInMovingRange(){
        return (int) (Timer.getFPGATimestamp() / cycleTime);
    }

    private int getFirstInMovingRange(int lengthOfStrip){
        return getFirstInMovingRange() % lengthOfStrip;
    }

    private void defineTheArrayOfTheColors(Color[] colors, int firstInMovingRange, int lastInMovingRange) {
        for (int i = 0; i < ledStrip.getLength(); i++) {
            if (shouldBePrimeColor(firstInMovingRange, lastInMovingRange, i))
                colors[i] = primeColor;
            else
                colors[i] = backgroundColor;
        }
    }

    private boolean shouldBePrimeColor(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return isPositionInLedInRange(firstInMovingRange, lastInMovingRange, positionInLED) ||
                isSplitByEnd(lastInMovingRange, positionInLED);
    }

    private boolean isPositionInLedInRange(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return positionInLED >= firstInMovingRange && positionInLED <= lastInMovingRange;
    }

     private boolean isSplitByEnd(int lastInMovingRange, int positionInLED) {
         return positionInLED >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                 positionInLED <= lastInMovingRange % ledStrip.getLength();
     }
}

package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

import java.util.Arrays;


public class PingPongLedCommand extends LedCommand {
    private final Color backgroundColor;
    private final Color primeColor;
    private final double cycleTime;
    private final int amountOfMovingLeds;
    private final LedStrip ledStrip;

    /**
     * Constructs a new MovingColorsLEDCommand.
     *
     * @param backgroundColor    The color of the background
     * @param primeColor         The color of the moving leds
     * @param cycleTime          The time it takes for the moving leds to go from one pixel to the other
     * @param amountOfMovingLeds The amount of leds that are moving
     * @param ledStrip           The led strip
     */
    public PingPongLedCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip) {
        super(ledStrip);
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds;
        this.ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        int firstInMovingRange = getFirstInMovingRange(ledStrip.getLength());
        int lastInMovingRange = firstInMovingRange + amountOfMovingLeds;
        Arrays.fill(colors, backgroundColor);
        Arrays.fill(colors, firstInMovingRange, lastInMovingRange, primeColor);
        setLeds(colors);
    }

    private int getCycleCount() {
        return (int) (Timer.getFPGATimestamp() / cycleTime);
    }

    private int getMovingCount(int movingRange) {
        return getCycleCount() % (movingRange * 2);
    }

    private int getFirstInMovingRange(int lengthOfStrip){
        int movingRange = ledStrip.getLength() - amountOfMovingLeds;
        int movingCount = getMovingCount(movingRange);
        int firstInMovingRange = movingCount % movingRange;
        if (movingRange < movingCount)
            firstInMovingRange = movingRange - (movingCount - movingRange);
        return firstInMovingRange;
    }
}

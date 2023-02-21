package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class MoveBothSidesLEDCommand extends LedCommand {
    private final Color backgroundColor;
    private final Color primeColor;
    private final double cycleTime;
    private final int amountOfMovingLeds;
    private final LedStrip ledStrip;

    /**
     * @param backgroundColor    The color of the background
     * @param primeColor         The color of the moving leds
     * @param cycleTime          The time it takes for the moving leds to go from one pixel to the other
     * @param amountOfMovingLeds The amount of leds that are moving
     * @param ledStrip           The led strip
     */
    public MoveBothSidesLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip) {
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
        int lastInMovingRange = (firstInMovingRange + amountOfMovingLeds) % ledStrip.getLength();
        defineColors(colors, firstInMovingRange, lastInMovingRange);
        ledStrip.setLedsColors(colors);
    }

    private int getFirstInMovingRange(){
        return (int) ((Timer.getFPGATimestamp() / cycleTime) * 2);
    }

    private int getFirstInMovingRange(int lengthOfStrip){
        return (getFirstInMovingRange() % lengthOfStrip);
    }

    private boolean shouldBePrimeColor(int index, int firstInMovingRange, int lastInMovingRange) {
        return (index >= firstInMovingRange && index <= lastInMovingRange) ||
                (index >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                        index <= lastInMovingRange % ledStrip.getLength()) || ((ledStrip.getLength() - index) >= firstInMovingRange && (ledStrip.getLength() - index) <= lastInMovingRange) ||
                ((ledStrip.getLength() - index) >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                        (ledStrip.getLength() - index) <= lastInMovingRange % ledStrip.getLength());
    }

    private void defineColors(Color[] colors, int firstInMovingRange, int lastInMovingRange) {
        for (int j = 0; j < ledStrip.getLength(); j++) {
            if (shouldBePrimeColor(firstInMovingRange, lastInMovingRange, j)) {
                colors[(ledStrip.getLength() - j) % ledStrip.getLength()] = primeColor;
                colors[j] = primeColor;
            } else {
                colors[(ledStrip.getLength() - j) % ledStrip.getLength()] = backgroundColor;
                colors[j] = backgroundColor;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++)
            colors[i] = Color.kBlack;
        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class MovingAndStopInMiddleLEDCommand extends LedCommand {
    Color backgroundColor;
    Color primeColor;
    double cycleTime;
    int amountOfMovingLeds;
    private final LedStrip ledStrip;

    public MovingAndStopInMiddleLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip) {
        super(ledStrip);
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds - 1;
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        int firstInMovingRange = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % (ledStrip.getLength() / 2);
        int lastInMovingRange = (firstInMovingRange + amountOfMovingLeds) % ledStrip.getLength();
        DefineTheArrayOfTheColors(colors, firstInMovingRange, lastInMovingRange);
        ledStrip.setLedsColors(colors);
    }

    private boolean shouldBePrimeColor(int index, int firstInMovingRange, int lastInMovingRange) {
        return (index >= firstInMovingRange && index <= lastInMovingRange) ||
                (index >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds && index <= lastInMovingRange % ledStrip.getLength()) ||
                ((ledStrip.getLength() - index) >= firstInMovingRange && (LedsConstants.LEDS_LENGTH - index) <= lastInMovingRange) ||
                ((ledStrip.getLength() - index) >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                        (ledStrip.getLength() - index) <= lastInMovingRange % ledStrip.getLength());
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

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++) {
            colors[i] = Color.kBlack;
        }
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
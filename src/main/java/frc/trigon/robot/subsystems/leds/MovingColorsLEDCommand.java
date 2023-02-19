package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MovingColorsLEDCommand extends LedCommand {
    Color backgroundColor;
    Color primeColor;
    double cycleTime;
    int amountOfMovingLeds;
    private final LedStrip ledStrip;

    public MovingColorsLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip) {
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
        int firstInMovingRange = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % ledStrip.getLength();
        int lastInMovingRange = firstInMovingRange + amountOfMovingLeds;
        for (int i = 0; i < ledStrip.getLength(); i++) {
            if (shouldBePrimeColor(firstInMovingRange, lastInMovingRange, i))
                colors[i] = primeColor;
            else
                colors[i] = backgroundColor;
        }

        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++) {
            colors[i] = Color.kBlack;
        }
        ledStrip.setLedsColors(colors);
    }

    private boolean shouldBePrimeColor(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return (positionInLED >= firstInMovingRange && positionInLED <= lastInMovingRange) ||
                (positionInLED >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                        positionInLED <= lastInMovingRange % ledStrip.getLength());
    }
}

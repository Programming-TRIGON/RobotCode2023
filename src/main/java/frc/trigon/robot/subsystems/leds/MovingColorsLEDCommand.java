package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MovingColorsLEDCommand extends CommandBase {
    Color backgroundColor;
    Color primeColor;
    double cycleTime;
    int amountOfMovingLeds;
    private final LedStrip ledStrip;

    public MovingColorsLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds, LedStrip ledStrip) {
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds - 1;
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {

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

    private boolean shouldBePrimeColor(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        boolean result;
        if ((positionInLED >= firstInMovingRange && positionInLED <= lastInMovingRange) ||
                (positionInLED >= lastInMovingRange % ledStrip.getLength() - amountOfMovingLeds &&
                        positionInLED <= lastInMovingRange % ledStrip.getLength())) {
            result = true;
        } else {
            result = false;
        }
        return result;
    }
}

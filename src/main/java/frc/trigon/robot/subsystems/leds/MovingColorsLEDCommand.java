package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MovingColorsLEDCommand extends CommandBase {

    Color backgroundColor;
    Color primeColor;
    double cycleTime;
    int amountOfMovingLeds;

    public MovingColorsLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds) {
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds =amountOfMovingLeds - 1;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        int firstInMovingRange = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % LedsConstants.LEDS_LENGTH;
        int lastInMovingRange = firstInMovingRange + amountOfMovingLeds;

        for (int i = 0; i < LedsConstants.LEDS_LENGTH; i++)
            colors[i] = (i >= firstInMovingRange && i <= lastInMovingRange) ||
                    (i >= lastInMovingRange % LedsConstants.LEDS_LENGTH - amountOfMovingLeds &&
                            i <= lastInMovingRange % LedsConstants.LEDS_LENGTH)
                    ? primeColor : backgroundColor;

        Leds.getInstance().setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}

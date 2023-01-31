package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MovingBothSidesColorsLEDCommand extends CommandBase {
    Color backgroundColor;
    Color primeColor;
    double cycleTime;
    int amountOfMovingLeds;

    public MovingBothSidesColorsLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int amountOfMovingLeds){
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds -1;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        int firstInMovingRange = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % LedsConstants.LEDS_LENGTH;
        int lastInMovingRange = (firstInMovingRange + amountOfMovingLeds) % LedsConstants.LEDS_LENGTH;

        for (int i = 0; i < LedsConstants.LEDS_LENGTH; i++) {

            if ((i >= firstInMovingRange && i <= lastInMovingRange) ||
                    (i >= lastInMovingRange % LedsConstants.LEDS_LENGTH - amountOfMovingLeds &&
                            i <= lastInMovingRange % LedsConstants.LEDS_LENGTH) || ((LedsConstants.LEDS_LENGTH - i) >= firstInMovingRange && (LedsConstants.LEDS_LENGTH - i) <= lastInMovingRange) ||
                    ((LedsConstants.LEDS_LENGTH - i) >= lastInMovingRange % LedsConstants.LEDS_LENGTH - amountOfMovingLeds &&
                            (LedsConstants.LEDS_LENGTH - i) <= lastInMovingRange % LedsConstants.LEDS_LENGTH)){
                colors[(LedsConstants.LEDS_LENGTH - i) % LedsConstants.LEDS_LENGTH] = primeColor;
                colors[i] = primeColor;
            }else {
                colors[(LedsConstants.LEDS_LENGTH - i) % LedsConstants.LEDS_LENGTH] = backgroundColor;
                colors[i] = backgroundColor;
            }
        }
        Leds.getInstance().setLedsColors(colors);

    }

    @Override
    public void end(boolean interrupted) {
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

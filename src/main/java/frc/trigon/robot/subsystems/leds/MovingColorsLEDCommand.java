package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MovingColorsLEDCommand extends CommandBase {

    Color backgroundColor;
    Color primeColor;
    double cycleTime;
    int numOfLeds;

    public MovingColorsLEDCommand(Color backgroundColor, Color primeColor, double cycleTime, int numOfLeds) {
        this.backgroundColor = backgroundColor;
        this.primeColor = primeColor;
        this.cycleTime = cycleTime;
        this.numOfLeds =numOfLeds - 1;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        int primeLED = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % LedsConstants.LEDS_LENGTH;
        int lastOnLED = primeLED + numOfLeds;
        for (int i = 0; i < LedsConstants.LEDS_LENGTH; i++)
            colors[i] = (i >= primeLED && i <= lastOnLED) ||
                    (i >= lastOnLED % LedsConstants.LEDS_LENGTH - numOfLeds &&
                            i <= lastOnLED % LedsConstants.LEDS_LENGTH)
                    ? primeColor : backgroundColor;
        Leds.getInstance().setLedsColors(colors);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Arrays;

public class BlinkLEDCommand extends CommandBase {
    private final Color[] theColors;
    private final double cycleTime;
    private final LedStrip ledStrip;
    public BlinkLEDCommand(Color[] theColors, double cycleTime, LedStrip ledStrip){
        this.theColors = theColors;
        this.cycleTime = cycleTime * 2;
        this.ledStrip = ledStrip;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++) {
            colors[i] = theColors[(int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % theColors.length];
        }


        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

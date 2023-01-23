package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Arrays;

public class BlinkLEDCommand extends CommandBase {
    private final Color color;
    private final double cycleTime;

    public BlinkLEDCommand(Color color, double cycleTime){
        this.color = color;
        this.cycleTime = cycleTime;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        if ((Timer.getFPGATimestamp() / cycleTime)% 1 <= 0.5) {
            Arrays.fill(colors, color);
        } else {
            Arrays.fill(colors, Color.kBlack);
        }
        Leds.getInstance().setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

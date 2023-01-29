package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Arrays;

public class BlinkLEDCommand extends CommandBase {
    private final Color color1;
    private final Color color2;

    private final double cycleTime;

    public BlinkLEDCommand(Color color1, Color color2, double cycleTime){
        this.color1 = color1;
        this.color2 = color2;
        this.cycleTime = cycleTime;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        if ((Timer.getFPGATimestamp() / cycleTime)% 1 <= 0.5) {
            Arrays.fill(colors, color1);
        } else {
            Arrays.fill(colors, color2);
        }
        Leds.getInstance().setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

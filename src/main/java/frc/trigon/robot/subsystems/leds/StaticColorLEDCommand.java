package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Arrays;

public class StaticColorLEDCommand extends CommandBase {
    private final Color color;
    private final LedStrip ledStrip;
    public StaticColorLEDCommand(Color color, LedStrip ledStrip) {
        this.color = color;
        this.ledStrip = ledStrip;
    }


    @Override
    public void initialize() {
        Color[] colors = new Color[ledStrip.getLength()];
        Arrays.fill(colors, color);
        ledStrip.setLedsColors(colors);
    }

    @Override
    public void execute() {
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

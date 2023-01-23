package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

import java.util.Arrays;

public class StaticColorLEDCommand extends CommandBase {
    private final Color color;

    public StaticColorLEDCommand(Color color) {this.color = color;}


    @Override
    public void initialize() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        Arrays.fill(colors, color);
        Leds.getInstance().setLedsColors(colors);
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

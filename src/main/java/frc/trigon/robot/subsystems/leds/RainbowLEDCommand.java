package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowLEDCommand extends CommandBase {

    private final LedStrip ledStrip;


    public RainbowLEDCommand(LedStrip ledStrip) {
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < colors.length; i++) {
            final int hue = (i * 180 / colors.length) % 180;
            colors[i] = Color.fromHSV(hue, 255, 128);
        }
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

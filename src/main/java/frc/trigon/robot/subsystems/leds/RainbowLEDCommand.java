package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowLEDCommand extends LedCommand {

    private final LedStrip ledStrip;

    /**
     * @param ledStrip the led strip
     */
    public RainbowLEDCommand(LedStrip ledStrip) {
        super(ledStrip);
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {
        super.initialize();
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < colors.length; i++) {
            final int hue = (i * 180 / colors.length) % 180;
            colors[i] = Color.fromHSV(hue, 255, 128);
        }
        ledStrip.setLedsColors(colors);
    }

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++)
            colors[i] = Color.kBlack;
        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkLEDCommand extends LedCommand {
    private final Color[] colors;
    private final double cycleTime;
    private final LedStrip ledStrip;

    /**
     * @param ledStrip  the led strip
     * @param colors    the colors of the led strip
     * @param cycleTime the time that takes to move from one pixel to the other
     */
    public BlinkLEDCommand(Color[] colors, double cycleTime, LedStrip ledStrip) {
        super(ledStrip);
        this.colors = colors;
        this.cycleTime = cycleTime * 2;
        this.ledStrip = ledStrip;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        int wantedColor = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % colors.length;
        for (int i = 0; i < ledStrip.getLength(); i++)
            colors[i] = this.colors[wantedColor];
        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++)
            colors[i] = Color.kBlack;
        ledStrip.setLedsColors(colors);
    }
}

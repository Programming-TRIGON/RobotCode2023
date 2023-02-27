package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

public class BlinkLEDCommand extends LedCommand {
    private final Color[] colors;
    private final double cycleTime;
    private final LedStrip ledStrip;

    /**
     * Constructs a new BlinkLEDCommand.
     *
     * @param colors    the colors of the led strip
     * @param cycleTime the time that takes to move from one pixel to the other
     * @param ledStrip  the led strip
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
        int wantedColor = (int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % this.colors.length;
        for (int i = 0; i < ledStrip.getLength(); i++)
            colors[i] = this.colors[wantedColor];
        setLeds(colors);
    }
}

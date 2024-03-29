package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

public class StaticRainbowLedCommand extends LedCommand {
    private final LedStrip ledStrip;
    private static final int saturation = 255;
    private static final int value = 128;

    /**
     * Constructs a new RainbowLEDCommand.
     *
     * @param ledStrip the led strip
     */
    public StaticRainbowLedCommand(LedStrip ledStrip) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void initialize() {
        super.initialize();
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < colors.length; i++) {
            final int hue = (i * 180 / colors.length) % 180;
            colors[i] = Color.fromHSV(hue, saturation, value);
        }
        setLeds(colors);
    }
}

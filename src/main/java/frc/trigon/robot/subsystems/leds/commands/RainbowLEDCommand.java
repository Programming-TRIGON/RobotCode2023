package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

public class RainbowLEDCommand extends LedCommand {
    private final LedStrip ledStrip;
    private final int saturation = 255;
    private final int value = 128;

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
            colors[i] = Color.fromHSV(hue, saturation, value);
        }
        setLeds(colors);
    }
}

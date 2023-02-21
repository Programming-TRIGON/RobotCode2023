package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class StaticColorLEDCommand extends LedCommand {
    private final LedStrip ledStrip;
    private final Color[] colors;
    private final int[] lengthOfEveryStrip;

    /**
     * @param ledStrip the led strip
     * @param colors the colors of the led strip
     * @param lengthOfEveryStrip the length of every strip
     */
    public StaticColorLEDCommand(LedStrip ledStrip, Color[] colors, int[] lengthOfEveryStrip) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        this.colors = colors;
        this.lengthOfEveryStrip = lengthOfEveryStrip;
    }

    @Override
    public void initialize() {
        super.initialize();
        Color[] colors = new Color[ledStrip.getLength()];
        boolean end = false;
        int counter = 0;
        while (!end) {
            for (int i = 0; i < this.colors.length && i < lengthOfEveryStrip.length; i++) {
                for (int j = 0; j < lengthOfEveryStrip[i]; j++) {
                    if (counter >= ledStrip.getLength()) {
                        end = true;
                        break;
                    }
                    colors[counter] = this.colors[i];
                    counter++;
                }
            }
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

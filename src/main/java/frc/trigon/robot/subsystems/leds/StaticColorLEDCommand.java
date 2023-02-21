package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class StaticColorLEDCommand extends LedCommand {
    private final LedStrip ledStrip;
    private final Color[] colors;
    private final int[] lengthOfEveryLedGroup;

    /**
     * @param ledStrip the led strip
     * @param colors the colors of the led strip
     * @param lengthOfEveryLedGroup the length of every strip
     */
    public StaticColorLEDCommand(LedStrip ledStrip, Color[] colors, int[] lengthOfEveryLedGroup) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        this.colors = colors;
        this.lengthOfEveryLedGroup = lengthOfEveryLedGroup;
    }

    @Override
    public void initialize() {
        super.initialize();
        Color[] colors = new Color[ledStrip.getLength()];
        ledStrip.setLedsColors(setTheColorsArray(colors));
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

    private Color[] setTheColorsArray(Color[] colors) {
        boolean end = false;
        int counter = 0;
        while (!end) {
            for (int i = 0; i < this.colors.length && i < lengthOfEveryLedGroup.length; i++) {
                for (int j = 0; j < lengthOfEveryLedGroup[i]; j++) {
                    if (counter >= ledStrip.getLength()) {
                        end = true;
                        break;
                    }
                    colors[counter] = this.colors[i];
                    counter++;
                }
            }
        }
        return colors;
    }
}

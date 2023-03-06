package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

public class StaticColorLedCommand extends LedCommand {
    private final LedStrip ledStrip;
    private final Color[] colors;
    private final int[] lengthOfEveryLedGroup;

    /**
     * Constructs a new StaticColorLEDCommand.
     *
     * @param ledStrip              the led strip
     * @param colors                the colors of the led strip
     * @param lengthOfEveryLedGroup the length of every strip
     */
    public StaticColorLedCommand(LedStrip ledStrip, Color[] colors, int[] lengthOfEveryLedGroup) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        this.colors = colors;
        this.lengthOfEveryLedGroup = lengthOfEveryLedGroup;
        addRequirements(ledStrip);
    }

    public StaticColorLedCommand(LedStrip ledStrip, Color color) {
        this(ledStrip, new Color[]{color}, new int[]{1});
    }

    @Override
    public void initialize() {
        super.initialize();
        Color[] colors = new Color[ledStrip.getLength()];
        setLeds(setColorsArray(colors));
    }

    private Color[] setColorsArray(Color[] colors) {
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

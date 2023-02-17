package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.reflect.Array;


public class StaticColorLEDCommand extends LedCommand {
    private final LedStrip ledStrip;
    private final Color[] theColors;
    private final int[] lengthOfEveryStrip;

    public StaticColorLEDCommand(LedStrip ledStrip, Color[] theColors, int[] lengthOfEveryStrip) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        this.theColors = theColors;
        this.lengthOfEveryStrip = lengthOfEveryStrip;

    }



    @Override
    public void initialize() {
        super.initialize();
        Color[] colors = new Color[ledStrip.getLength()];
        boolean end = false;
        int counter = 0;
        while (!end) {
            for (int i = 0; i < theColors.length && i < lengthOfEveryStrip.length; i++) {
                for (int j = 0; j < lengthOfEveryStrip[i]; j++) {
                    if (counter >= ledStrip.getLength()) {
                        end = true;
                        break;
                    }
                    colors[counter] = theColors[i];
                    counter++;
                }
            }
        }
        ledStrip.setLedsColors(colors);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++){
            colors[i] = Color.kBlack;
        }
        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

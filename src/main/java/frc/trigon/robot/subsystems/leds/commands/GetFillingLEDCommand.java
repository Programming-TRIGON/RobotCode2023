package frc.trigon.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedStrip;

import java.lang.reflect.Array;
import java.util.Arrays;

public class GetFillingLEDCommand extends LedCommand {

    private final LedStrip ledStrip;
    private final Color backgroundColor;
    private final Color fillingColor;
    private final double cycleTime;

    private boolean isSecondColor = false;

    /**
     * Constructs a new LedCommand.
     *
     * @param ledStrip        the led strip
     * @param backgroundColor
     * @param fillingColor
     * @param cycleTime
     */
    public GetFillingLEDCommand(LedStrip ledStrip, Color backgroundColor, Color fillingColor, double cycleTime) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        this.backgroundColor = backgroundColor;
        this.fillingColor = fillingColor;
        this.cycleTime = cycleTime;
        addRequirements(ledStrip);
    }


    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        int index = getCurrentIndex();
        Arrays.fill(colors, otherColor());
        for (int i = 0; i < ledStrip.getLength(); i++) {
            if (i <= index)
                colors[i] = currentColor();
        }
        if (index == ledStrip.getLength())
            isSecondColor = !isSecondColor;
        setLeds(colors);
    }

    private int getCurrentIndex() {
        return (int) ((Timer.getFPGATimestamp() / cycleTime) % ledStrip.getLength());
    }

    private Color currentColor() {
        if (isSecondColor)
            return fillingColor;
        else
            return backgroundColor;
    }

    private Color otherColor() {
        if (isSecondColor)
            return backgroundColor;
        else
            return fillingColor;
    }
}

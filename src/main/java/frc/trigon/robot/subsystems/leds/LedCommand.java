package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Arrays;

public abstract class LedCommand extends CommandBase {
    private final LedStrip ledStrip;

    /**
     * @param ledStrip the led strip
     */
    protected LedCommand(LedStrip ledStrip) {
        this.ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void initialize() {
        ledStrip.cancelOverlapping();
    }

    private void turnOffLeds() {
        Color[] colors = new Color[ledStrip.getLength()];
        Arrays.fill(colors, Color.kBlack);
        ledStrip.setLedsColors(colors);
    }

    protected void setLeds(Color[] colors) {
        ledStrip.setLedsColors(colors);
    }

    @Override
    public void end(boolean interrupted) {
        turnOffLeds();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
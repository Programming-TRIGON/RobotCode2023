package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkLEDCommand extends LedCommand {
    private final Color[] theColors;
    private final double cycleTime;
    private final LedStrip ledStrip;
    public BlinkLEDCommand(Color[] theColors, double cycleTime, LedStrip ledStrip){
        super(ledStrip);
        this.theColors = theColors;
        this.cycleTime = cycleTime * 2;
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++) {
            colors[i] = theColors[(int) ((Timer.getFPGATimestamp() / cycleTime) * 2) % theColors.length];
        }


        ledStrip.setLedsColors(colors);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < ledStrip.getLength(); i++){
            colors[i] = Color.kBlack;
        }
        ledStrip.setLedsColors(colors);
    }
}

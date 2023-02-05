package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TwoStaticColorsLEDCommand extends CommandBase {

    Color color1;
    Color color2;
    private final LedStrip ledStrip;



    public TwoStaticColorsLEDCommand(Color color1, Color color2, LedStrip ledStrip) {
        this.color1 = color1;
        this.color2 = color2;
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {
        Color[] colors = new Color[ledStrip.getLength()];
        for (int i = 0; i < colors.length; i++){
            if (i % 2 == 0){
                colors[i] = color1;
            }else colors[i] = color2;
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
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

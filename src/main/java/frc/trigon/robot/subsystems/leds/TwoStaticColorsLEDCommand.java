package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Arrays;


public class TwoStaticColorsLEDCommand extends CommandBase {

    Color color1;
    Color color2;

    public TwoStaticColorsLEDCommand(Color color1, Color color2) {
        this.color1 = color1;
        this.color2 = color2;
    }

    @Override
    public void initialize() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        for (int i = 0; i < LedsConstants.LEDS_LENGTH; i++){
            if (i % 2 == 0){
                colors[i] = color1;
            }else colors[i] = color2;
        }
        Leds.getInstance().setLedsColors(colors);
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

package frc.trigon.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.subsystems.leds.*;


public class RobotContainer {
    public RobotContainer() {
        Command ss = new StaticColorLEDCommand(new LedStrip(0, 20), new Color[]{Color.kCadetBlue, Color.kLime, Color.kRed}, new int[]{2,4,2});
        ss.schedule();
    }
}

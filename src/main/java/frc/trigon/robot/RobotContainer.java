package frc.trigon.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.subsystems.leds.Leds;


public class RobotContainer {
    public RobotContainer() {
        Command turnToRed = Leds.getInstance().StaticColorCommand(new Color(255,10,120));
        turnToRed.schedule();
        //Command rainbow = Leds.getInstance().CommandRainbow();
        //rainbow.schedule();
        SmartDashboard.putData(Leds.getInstance());
    }
}

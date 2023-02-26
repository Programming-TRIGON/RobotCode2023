package frc.trigon.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.leds.commands.MovingColorsLEDCommand;
import frc.trigon.robot.subsystems.leds.commands.PingPongLEDCommand;

public class RobotContainer {
    public RobotContainer() {
        Command command = new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, new LedStrip(0, 80, false));
        command.schedule();
    }
}
package frc.trigon.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.subsystems.leds.*;


public class RobotContainer {
    public RobotContainer() {


        SmartDashboard.putData(Leds.getInstance());
    }
}

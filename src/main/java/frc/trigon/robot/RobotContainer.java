package frc.trigon.robot;

import frc.trigon.robot.componenets.XboxController;

public class RobotContainer {
    private final double driveControllerDeadband = 0.1;
    private final XboxController driveController = new XboxController(
            0, true,
            driveControllerDeadband,
            edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger
    );

    public RobotContainer() {
    }
}

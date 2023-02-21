package frc.trigon.robot;

import frc.trigon.robot.components.XboxController;

public class DriverConstants {
    private static final int DRIVE_CONTROLLER_PORT = 0;
    private static final int DRIVE_CONTROLLER_EXPONENT = 2;
    private static final double DRIVE_CONTROLLER_DEADBAND = 0.05;
    static final XboxController DRIVE_CONTROLLER = new XboxController(
            DRIVE_CONTROLLER_PORT, DRIVE_CONTROLLER_EXPONENT, DRIVE_CONTROLLER_DEADBAND
    );
}

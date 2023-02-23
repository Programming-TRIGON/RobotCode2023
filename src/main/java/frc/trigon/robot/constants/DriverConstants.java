package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.XboxController;

public class DriverConstants {
    private static final int DRIVE_CONTROLLER_PORT = 0;
    private static final int DRIVE_CONTROLLER_EXPONENT = 2;
    private static final double DRIVE_CONTROLLER_DEADBAND = 0.1;
    public static final XboxController DRIVE_CONTROLLER = new XboxController(
            DRIVE_CONTROLLER_PORT, DRIVE_CONTROLLER_EXPONENT, DRIVE_CONTROLLER_DEADBAND
    );
    public static final double POV_DIVIDER = 5;

    public static final Trigger
            RESET_POSE_TRIGGER = DRIVE_CONTROLLER.y(),
            TOGGLE_FIELD_AND_SELF_DRIVEN_ANGLE_TRIGGER = DRIVE_CONTROLLER.x(),
            LOCK_SWERVE_TRIGGER = DRIVE_CONTROLLER.a(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVE_CONTROLLER.getPov() != -1),
            RT_TRIGGER = DRIVE_CONTROLLER.leftBumper();
}

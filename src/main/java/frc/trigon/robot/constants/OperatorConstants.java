package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.XboxController;

public class OperatorConstants {
    private static final int DRIVE_CONTROLLER_PORT = 0;
    private static final int DRIVE_CONTROLLER_EXPONENT = 2;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.1;
    public static final XboxController DRIVE_CONTROLLER = new XboxController(
            DRIVE_CONTROLLER_PORT, DRIVE_CONTROLLER_EXPONENT, DRIVE_CONTROLLER_DEADBAND
    );
    public static final double
            POV_DIVIDER = 2,
            STICKS_DIVIDER = 1.8;
    private static final double MINIMUM_SHIFT_VALUE = 0.18;
    public static final double MINIMUM_SHIFT_VALUE_COEFFICIENT = 1 - (1 / MINIMUM_SHIFT_VALUE);
    public static final CommandGenericHID KEYBOARD_INPUT = new CommandGenericHID(1);

    public static final Trigger
            RESET_POSE_TRIGGER = DRIVE_CONTROLLER.y(),
            TOGGLE_FIELD_AND_SELF_DRIVEN_ANGLE_TRIGGER = DRIVE_CONTROLLER.x(),
            LOCK_SWERVE_TRIGGER = DRIVE_CONTROLLER.a(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVE_CONTROLLER.getPov() != -1),
            LEVEL_1_TRIGGER = KEYBOARD_INPUT.button(1),
            LEVEL_2_TRIGGER = KEYBOARD_INPUT.button(2),
            LEVEL_3_TRIGGER = KEYBOARD_INPUT.button(3),
            CONE_TRIGGER = KEYBOARD_INPUT.button(4),
            CUBE_TRIGGER = KEYBOARD_INPUT.button(5),
            GRID_1_TRIGGER = KEYBOARD_INPUT.button(6),
            GRID_2_TRIGGER = KEYBOARD_INPUT.button(7),
            GRID_3_TRIGGER = KEYBOARD_INPUT.button(8),
            LEFT_RAMP_TRIGGER = KEYBOARD_INPUT.button(9),
            RIGHT_RAMP_TRIGGER = KEYBOARD_INPUT.button(10),
            ALIGN_TO_GRID_TRIGGER = KEYBOARD_INPUT.button(11),
            APPLY_FIRST_ARM_STATE_TRIGGER = KEYBOARD_INPUT.button(12),
            APPLY_SECOND_ARM_STATE_TRIGGER = KEYBOARD_INPUT.button(13),
            EJECT_TRIGGER = KEYBOARD_INPUT.button(14),
            START_AUTO_TRIGGER = KEYBOARD_INPUT.button(15);
}

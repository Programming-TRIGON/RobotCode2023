package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.utilities.KeyboardController;

public class OperatorConstants {
    private static final int DRIVE_CONTROLLER_PORT = 0;
    private static final int DRIVE_CONTROLLER_EXPONENT = 2;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.1;
    public static final XboxController DRIVE_CONTROLLER = new XboxController(
            DRIVE_CONTROLLER_PORT, DRIVE_CONTROLLER_EXPONENT, DRIVE_CONTROLLER_DEADBAND
    );
    public static final double
            POV_DIVIDER = 2,
            STICKS_DIVIDER = 1;
    private static final double MINIMUM_SHIFT_VALUE = 0.18;
    public static final double MINIMUM_SHIFT_VALUE_COEFFICIENT = 1 - (1 / MINIMUM_SHIFT_VALUE);
    public static final KeyboardController KEYBOARD_CONTROLLER = new KeyboardController(1);

    public static final Trigger
            RESET_POSE_TRIGGER = DRIVE_CONTROLLER.y(),
            TURN_TO_GRID_TRIGGER = DRIVE_CONTROLLER.x(),
            LOCK_SWERVE_TRIGGER = DRIVE_CONTROLLER.a(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVE_CONTROLLER.getPov() != -1),
            GO_AND_PLACE_TRIGGER = new Trigger(),
            SELF_RELATIVE_DRIVE_TRIGGER = DRIVE_CONTROLLER.b(),
            LEVEL_1_TRIGGER = KEYBOARD_CONTROLLER.numpad1(),
            LEVEL_2_TRIGGER = KEYBOARD_CONTROLLER.numpad2(),
            LEVEL_3_TRIGGER = KEYBOARD_CONTROLLER.numpad3(),
            GRID_1_TRIGGER = KEYBOARD_CONTROLLER.numpad4(),
            GRID_2_TRIGGER = KEYBOARD_CONTROLLER.numpad5(),
            GRID_3_TRIGGER = KEYBOARD_CONTROLLER.numpad6(),
            LEFT_RAMP_TRIGGER = KEYBOARD_CONTROLLER.minus(),
            RIGHT_RAMP_TRIGGER = KEYBOARD_CONTROLLER.equals(),
            ALIGN_TO_GRID_TRIGGER = KEYBOARD_CONTROLLER.up(),
            APPLY_FIRST_ARM_STATE_TRIGGER = KEYBOARD_CONTROLLER.left(),
            APPLY_SECOND_ARM_STATE_TRIGGER = KEYBOARD_CONTROLLER.right(),
            START_AUTO_TRIGGER = KEYBOARD_CONTROLLER.rightAlt(),
            LED_FLAMES_TRIGGER = KEYBOARD_CONTROLLER.numpad8(),
            PRELOAD_CURRENT_AUTO_TRIGGER = KEYBOARD_CONTROLLER.o(),
            HIGH_CONE_TRIGGER = KEYBOARD_CONTROLLER.w(),
            CONE_TRIGGER = KEYBOARD_CONTROLLER.e(),
            CUBE_TRIGGER = KEYBOARD_CONTROLLER.r(),
            CUBE_HIGH_TRIGGER = KEYBOARD_CONTROLLER.y(),
            CUBE_MIDDLE_TRIGGER = KEYBOARD_CONTROLLER.h(),
            CUBE_LOW_TRIGGER = KEYBOARD_CONTROLLER.n(),
            CONE_HIGH_TRIGGER = KEYBOARD_CONTROLLER.u(),
            CONE_MIDDLE_TRIGGER = KEYBOARD_CONTROLLER.j(),
            CONE_LOW_TRIGGER = KEYBOARD_CONTROLLER.m(),
            CONTINUE_PLACEMENT_TRIGGER = KEYBOARD_CONTROLLER.k().or(DRIVE_CONTROLLER.leftBumper()),
            EJECT_TRIGGER = KEYBOARD_CONTROLLER.comma(),
            BALANCE_TRIGGER = KEYBOARD_CONTROLLER.p();
}

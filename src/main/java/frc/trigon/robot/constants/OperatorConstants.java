package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.utilities.FilesHandler;
import frc.trigon.robot.utilities.KeyboardController;

public class OperatorConstants {
    public static String
            REAL_MODE_LOG_DIRECTORY = FilesHandler.DEPLOY_PATH + "logs/",
            SIMULATION_MODE_LOG_DIRECTORY = FilesHandler.DEPLOY_PATH + "logs/";
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
            RESET_HEADING_TRIGGER = DRIVE_CONTROLLER.y(),
            FIELD_RELATIVE_DRIVEN_ANGLE_TRIGGER = DRIVE_CONTROLLER.x(),
            LOCK_SWERVE_TRIGGER = DRIVE_CONTROLLER.a(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVE_CONTROLLER.getPov() != -1),
            DRIVE_AND_PLACE_TRIGGER = new Trigger(() -> false),
            SELF_RELATIVE_DRIVE_TRIGGER = DRIVE_CONTROLLER.b(),
            CLOSED_COLLECTING_TRIGGER = DRIVE_CONTROLLER.leftTrigger(),
            CLOSED_COLLECTING_STANDING_CONE_TRIGGER = DRIVE_CONTROLLER.rightBumper(),
            ALIGN_TO_REFLECTOR_WITH_CONTROLLER_TRIGGER = DRIVE_CONTROLLER.leftBumper(),
            COLLECT_FROM_FEEDER_WITH_MANUAL_DRIVE_TRIGGER = DRIVE_CONTROLLER.a(),
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
            PRELOAD_CURRENT_AUTO_TRIGGER = KEYBOARD_CONTROLLER.tab(),
            HIGH_CONE_TRIGGER = KEYBOARD_CONTROLLER.w(),
            MIDDLE_CONE_TRIGGER = KEYBOARD_CONTROLLER.e(),
            CUBE_TRIGGER = KEYBOARD_CONTROLLER.r(),
            CUBE_HIGH_TRIGGER = KEYBOARD_CONTROLLER.y(),
            CUBE_MIDDLE_TRIGGER = KEYBOARD_CONTROLLER.h(),
            CUBE_LOW_TRIGGER = KEYBOARD_CONTROLLER.n(),
            CONE_HIGH_TRIGGER = KEYBOARD_CONTROLLER.u(),
            CONE_MIDDLE_TRIGGER = KEYBOARD_CONTROLLER.j(),
            CONE_LOW_TRIGGER = KEYBOARD_CONTROLLER.m(),
            CONTINUE_PLACEMENT_TRIGGER = KEYBOARD_CONTROLLER.k().or(DRIVE_CONTROLLER.leftBumper()),
            EJECT_TRIGGER = KEYBOARD_CONTROLLER.comma(),
            BALANCE_TRIGGER = KEYBOARD_CONTROLLER.p(),
            ALIGN_TO_REFLECTOR_TRIGGER = KEYBOARD_CONTROLLER.a(),
            FULL_EJECT_TRIGGER = KEYBOARD_CONTROLLER.backtick(),
            USER_BUTTON_TRIGGER = KEYBOARD_CONTROLLER.g(),
            PLACE_CONE_AT_MID_TRIGGER = KEYBOARD_CONTROLLER.f8(),
            PLACE_CONE_AT_HIGH_TRIGGER = KEYBOARD_CONTROLLER.f9(),
            CLOSE_ARM_TRIGGER = KEYBOARD_CONTROLLER.numpad0(),
            GO_TO_TARGET_DASHBOARD_POSITION_TRIGGER = KEYBOARD_CONTROLLER.f7(),
            RESET_POSE_TO_BEFORE_CHARGE_STATION_TRIGGER = KEYBOARD_CONTROLLER.f10(),
            RESET_POSE_TO_AFTER_CHARGE_STATION_TRIGGER = KEYBOARD_CONTROLLER.f12();
}

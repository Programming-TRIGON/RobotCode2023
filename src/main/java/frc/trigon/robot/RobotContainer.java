package frc.trigon.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.AlignToReflectorCommand;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.CollectionCamera;
import frc.trigon.robot.components.ReflectionLimelight;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;

public class RobotContainer implements Loggable {
    public static final ReflectionLimelight REFLECTION_LIMELIGHT = new ReflectionLimelight("limelight");
    public static final CollectionCamera COLLECTION_CAMERA = new CollectionCamera("limelight-collection");

    // Subsystems TODO: make them not singletons
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    public static final Arm ARM = Arm.getInstance();
    public static final Gripper GRIPPER = Gripper.getInstance();
    public static final LedStrip LEDS = new LedStrip(63, false);
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();

    @Log(name = "autoChooser")
    public static final SendableChooser<String> AUTONOMOUS_PATH_NAME_CHOOSER = new SendableChooser<>();
    private final Trigger userButton = new Trigger(() -> RobotController.getUserButton() || OperatorConstants.USER_BUTTON_TRIGGER.getAsBoolean());

    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        configureAutonomousChooser();
        setPoseEstimatorPoseSources();
        bindCommands();
        PhotonCamera.setVersionCheckEnabled(false);

        SmartDashboard.putNumber("target1", SmartDashboard.getNumber("target1", 0));
        SmartDashboard.putNumber("target2", SmartDashboard.getNumber("target2", 0));

        configDriverCam();
    }

    public void teleopInit() {
        SWERVE.getDefaultCommand().schedule();
    }

    /**
     * @return the command to run in autonomous mode
     */
    CommandBase getAutonomousCommand() {
        if (AUTONOMOUS_PATH_NAME_CHOOSER.getSelected() == null)
            return new InstantCommand();

        return Commands.getAutonomousCommand(AUTONOMOUS_PATH_NAME_CHOOSER.getSelected());
    }

    private void bindCommands() {
        bindControllerCommands();
        bindDefaultCommands();
        setupArmBrakeModeWithUserButtonCommands();
    }

    private void bindControllerCommands() {
        configureTargetPlacingPositionSetters();

        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandsConstants.RESET_HEADING_COMMAND);
        OperatorConstants.FIELD_RELATIVE_DRIVEN_ANGLE_TRIGGER.whileTrue(CommandsConstants.FIELD_RELATIVE_DRIVEN_ANGLE_FROM_STICKS_COMMAND);
//        OperatorConstants.LOCK_SWERVE_TRIGGER.whileTrue(SwerveCommands.getLockSwerveCommand());
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandsConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.ALIGN_TO_GRID_TRIGGER.whileTrue(CommandsConstants.ALIGN_TO_GRID_COMMAND);
        OperatorConstants.APPLY_FIRST_ARM_STATE_TRIGGER.whileTrue(CommandsConstants.APPLY_FIRST_ARM_STATE_COMMAND);
        OperatorConstants.APPLY_SECOND_ARM_STATE_TRIGGER.whileTrue(CommandsConstants.APPLY_SECOND_ARM_STATE_COMMAND);
        OperatorConstants.EJECT_TRIGGER.whileTrue(Gripper.getInstance().getEjectCommand());
        OperatorConstants.START_AUTO_TRIGGER.whileTrue(new ProxyCommand(this::getAutonomousCommand));
        OperatorConstants.LED_FLAMES_TRIGGER.onTrue(CommandsConstants.FLAMES_LED_COMMAND);
        OperatorConstants.PRELOAD_CURRENT_AUTO_TRIGGER.onTrue(CommandsConstants.PRELOAD_CURRENT_AUTO_COMMAND);

        OperatorConstants.PLACE_CONE_AT_MID_TRIGGER.whileTrue(Commands.getPlaceConeAtMidCommand());
        OperatorConstants.PLACE_CONE_AT_HIGH_TRIGGER.whileTrue(Commands.getPlaceConeAtHighCommand());
        OperatorConstants.CLOSE_ARM_TRIGGER.whileTrue(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED));
        OperatorConstants.GO_TO_TARGET_DASHBOARD_POSITION_TRIGGER.whileTrue(CommandsConstants.GO_TO_TARGET_DASHBOARD_POSITION_COMMAND);

        OperatorConstants.DRIVE_AND_PLACE_TRIGGER.whileTrue(CommandsConstants.DRIVE_AND_PLACE_COMMAND);
        OperatorConstants.SELF_RELATIVE_DRIVE_TRIGGER.toggleOnTrue(CommandsConstants.SELF_RELATIVE_DRIVE_FROM_STICKS_COMMAND);
        OperatorConstants.CLOSED_COLLECTING_TRIGGER.whileTrue(CommandsConstants.CLOSED_COLLECTING_COMMAND);
        OperatorConstants.CLOSED_COLLECTING_STANDING_CONE_TRIGGER.whileTrue(CommandsConstants.CLOSED_COLLECTING_STANDING_CONE_COMMAND);
        OperatorConstants.COLLECT_FROM_FEEDER_WITH_MANUAL_DRIVE_TRIGGER.whileTrue(CommandsConstants.COLLECT_FROM_FEEDER_WITH_MANUAL_DRIVE_COMMAND);
        OperatorConstants.ALIGN_TO_REFLECTOR_WITH_CONTROLLER_TRIGGER.and(this::canAlignToReflector).whileTrue(new AlignToReflectorCommand());

        OperatorConstants.BALANCE_TRIGGER.whileTrue(SwerveCommands.getBalanceCommand());
        OperatorConstants.RESET_POSE_TO_BEFORE_CHARGE_STATION_TRIGGER.onTrue(CommandsConstants.RESET_POSE_TO_BEFORE_CHARGE_STATION_COMMAND);
        OperatorConstants.RESET_POSE_TO_AFTER_CHARGE_STATION_TRIGGER.onTrue(CommandsConstants.RESET_POSE_TO_AFTER_CHARGE_STATION_COMMAND);
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(CommandsConstants.FIELD_RELATIVE_OPEN_LOOP_SUPPLIER_DRIVE_COMMAND);
        ARM.setDefaultCommand(CommandsConstants.CLOSE_ARM_COMMAND);
        LEDS.setDefaultCommand(CommandsConstants.FLAMES_LED_COMMAND);
    }

    private void configureTargetPlacingPositionSetters() {
        OperatorConstants.LEVEL_1_TRIGGER.onTrue(CommandsConstants.SET_LEVEL_TO_ONE_COMMAND);
        OperatorConstants.LEVEL_2_TRIGGER.onTrue(CommandsConstants.SET_LEVEL_TO_TWO_COMMAND);
        OperatorConstants.LEVEL_3_TRIGGER.onTrue(CommandsConstants.SET_LEVEL_TO_THREE_COMMAND);

        OperatorConstants.MIDDLE_CONE_TRIGGER.onTrue(CommandsConstants.SET_TO_MIDDLE_CONE_COMMAND);
        OperatorConstants.HIGH_CONE_TRIGGER.onTrue(CommandsConstants.SET_TO_HIGH_CONE_COMMAND);
        OperatorConstants.CUBE_TRIGGER.onTrue(CommandsConstants.SET_TO_CUBE_COMMAND);

        OperatorConstants.GRID_1_TRIGGER.onTrue(CommandsConstants.SET_TO_GRID_1_COMMAND);
        OperatorConstants.GRID_2_TRIGGER.onTrue(CommandsConstants.SET_TO_GRID_2_COMMAND);
        OperatorConstants.GRID_3_TRIGGER.onTrue(CommandsConstants.SET_TO_GRID_3_COMMAND);

        OperatorConstants.LEFT_RAMP_TRIGGER.onTrue(CommandsConstants.SET_TO_LEFT_RAMP_COMMAND);
        OperatorConstants.RIGHT_RAMP_TRIGGER.onTrue(CommandsConstants.SET_TO_RIGHT_RAMP_COMMAND);

        OperatorConstants.CUBE_LOW_TRIGGER.whileTrue(Commands.getPlaceCubeAtHybridCommand());
        OperatorConstants.CUBE_MIDDLE_TRIGGER.whileTrue(Commands.getPlaceCubeAtMidCommand());
        OperatorConstants.CUBE_HIGH_TRIGGER.whileTrue(Commands.getPlaceCubeAtHighCommand());

        OperatorConstants.CONE_LOW_TRIGGER.whileTrue(Commands.getPlaceConeAtHybridCommand());
        OperatorConstants.CONE_MIDDLE_TRIGGER.whileTrue(Commands.getPlaceConeAtMidCommand());
        OperatorConstants.CONE_HIGH_TRIGGER.whileTrue(Commands.getPlaceConeAtHighCommand());

        OperatorConstants.ALIGN_TO_REFLECTOR_TRIGGER.whileTrue(new AlignToReflectorCommand());
        OperatorConstants.FULL_EJECT_TRIGGER.whileTrue(GRIPPER.getFullEjectCommand());
    }

    private void configureAutonomousChooser() {
        AUTONOMOUS_PATH_NAME_CHOOSER.setDefaultOption("None", null);

        for (String currentPathName : AutonomousConstants.AUTONOMOUS_PATHS_NAMES)
            AUTONOMOUS_PATH_NAME_CHOOSER.addOption(currentPathName, currentPathName);
    }

    private boolean canAlignToReflector() {
        return ARM.getDefaultCommand().equals(ARM.getCurrentCommand()) && ARM.atGoal();
    }

    private void setPoseEstimatorPoseSources() {
        poseEstimator.addRobotPoseSources(CameraConstants.FORWARD_LIMELIGHT);//,CameraConstants.t265);
    }

    private void setupArmBrakeModeWithUserButtonCommands() {
        userButton.onTrue(CommandsConstants.ARM_COAST_COMMAND);
        userButton.onFalse(CommandsConstants.ARM_BRAKE_COMMAND);
        userButton.whileTrue(CommandsConstants.ARM_COAST_WITH_PINK_LED_COMMAND);
    }

    private void configDriverCam() {
        final UsbCamera usbCamera = CameraServer.startAutomaticCapture(2);
        usbCamera.setResolution(424, 240);
        usbCamera.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    }
}

package frc.trigon.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.leds.commands.MovingColorsLedCommand;
import frc.trigon.robot.subsystems.leds.commands.StaticColorLedCommand;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.KeyboardController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;

import java.util.concurrent.atomic.AtomicReference;

import static frc.trigon.robot.subsystems.arm.ArmConstants.ArmStates;

public class RobotContainer implements Loggable {
    // Subsystems TODO: make them not singletons
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    public static final Arm ARM = Arm.getInstance();

    public static final Gripper GRIPPER = Gripper.getInstance();
    private final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();
    // private final CollectionCamera COLLECTION_CAM = new CollectionCamera("limelight-collection");

    @Log(name = "autoChooser")
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();

    // Triggers
    private final XboxController driverController = OperatorConstants.DRIVE_CONTROLLER;
    private final KeyboardController keyboardController = OperatorConstants.KEYBOARD_CONTROLLER;
    private final Trigger userButton = new Trigger(() -> RobotController.getUserButton() || keyboardController.g().getAsBoolean());
    private final Trigger tippingTrigger = new Trigger(() -> Math.abs(SWERVE.getPitch()) < -2);
    private final AtomicReference<Integer>
            level = new AtomicReference<>(1),
            grid = new AtomicReference<>(1);
    private final AtomicReference<Boolean>
            isCone = new AtomicReference<>(false),
            isLeftRamp = new AtomicReference<>(false);

    public static final LedStrip leds = new LedStrip(63, false);
    private final CommandBase
            fieldRelativeDriveFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
            () -> driverController.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
            () -> driverController.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
            () -> driverController.getRightX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue()
    ),
            selfRelativeDriveFromDpadCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> Math.sin(Units.degreesToRadians(-driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> 0
            ),
            resetHeadingCommand = new InstantCommand(
                    () -> POSE_ESTIMATOR.resetPose(setRotation(POSE_ESTIMATOR.getCurrentPose(), new Rotation2d()))
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDrivenAngle
            ),
            fieldRelativeDrivenAngleFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    this::getRightStickAsRotation2d
            ),
            alignToGridCommand = Commands.getDriveToPoseCommand(
                    new PathConstraints(1, 1),
                    () -> getGridAlignment().inFrontOfGridPose
            ),
            applyFirstArmStateCommand = getGoToCurrentFirstArmPositionCommand(),
            applySecondArmStateCommand = getGoToCurrentSecondArmPositionCommand(),
            placeGamePieceAtHybridCommand = ARM.getGoToStateCommand(ArmStates.HYBRID_1).alongWith(
                    new WaitCommand(0.8).until(ARM::atGoal).andThen(GRIPPER.getSlowEjectCommand())
            ),
            redClimbingLEDCommand = new MovingColorsLedCommand(leds, Color.kRed, 0.02, 5, Color.kBlack),
            flamesLEDCommand = new MovingColorsLedCommand(leds, new Color(0f, 0f, 1f), 0.02, 7, Color.kRed),
            staticYellowColorLedCommand = new MovingColorsLedCommand(leds, Color.kDarkBlue, 1, 0, Color.kYellow),
            staticPurpleColorLedCommand = new MovingColorsLedCommand(leds, Color.kDarkBlue, 1, 0, Color.kPurple);

    public RobotContainer() {
        configureAutonomousChooser();
        setPoseEstimatorPoseSources();
        bindCommands();
        PhotonCamera.setVersionCheckEnabled(false);
        keyboardController.f8().whileTrue(Commands.getPlaceConeAtMiddleCommand());
        keyboardController.f9().whileTrue(Commands.getPlaceCubeAtHighCommand());
//        new ProxyCommand(()->new PrintCommand(input.getHID().getRawAxis(0) +"")).repeatedly().schedule();
        keyboardController.numpad0().whileTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));
        keyboardController.f7().whileTrue(new ProxyCommand(() -> Arm.getInstance().getGoToPositionCommand(SmartDashboard.getNumber("target1", 0), SmartDashboard.getNumber("target2", 0), false).ignoringDisable(true)));
        SmartDashboard.putNumber("target1", SmartDashboard.getNumber("target1", 0));
        SmartDashboard.putNumber("target2", SmartDashboard.getNumber("target2", 0));
    }

    /**
     * @return the command to run in autonomous mode
     */
    CommandBase getAutonomousCommand() {
        if (autonomousPathNameChooser.getSelected() == null)
            return new InstantCommand();

        return Commands.getAutonomousCommand(autonomousPathNameChooser.getSelected());
    }

    private void bindCommands() {
        bindControllerCommands();
        bindDefaultCommands();
        setupArmBrakeModeWithUserButtonCommands();
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_POSE_TRIGGER.onTrue(resetHeadingCommand);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_DRIVEN_ANGLE_TRIGGER.onTrue(toggleFieldAndSelfDrivenCommand);
        OperatorConstants.LOCK_SWERVE_TRIGGER.whileTrue(SwerveCommands.getLockSwerveCommand());
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(selfRelativeDriveFromDpadCommand);
        OperatorConstants.ALIGN_TO_GRID_TRIGGER.whileTrue(alignToGridCommand);
        OperatorConstants.APPLY_FIRST_ARM_STATE_TRIGGER.whileTrue(applyFirstArmStateCommand);
        OperatorConstants.APPLY_SECOND_ARM_STATE_TRIGGER.whileTrue(applySecondArmStateCommand);
        OperatorConstants.EJECT_TRIGGER.whileTrue(Gripper.getInstance().getEjectCommand());
        OperatorConstants.START_AUTO_TRIGGER.whileTrue(new ProxyCommand(this::getAutonomousCommand));
        OperatorConstants.LED_FLAMES_TRIGGER.onTrue(flamesLEDCommand);
        OperatorConstants.PLACE_GAME_PIECE_AT_HYBRID_TRIGGER.whileTrue(placeGamePieceAtHybridCommand);
        tippingTrigger.onTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));

        driverController.leftTrigger().whileTrue(GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CLOSED_COLLECTING, true, 2)));
        driverController.rightBumper().whileTrue(GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CLOSED_COLLECTING_STANDING_CONE, true, 2)));
//        driverController.rightBumper().whileTrue(GRIPPER.getSlowCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CONE_FEEDER, true, 0.5)));
        driverController.leftBumper().whileTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));

        configureTargetPlacingPositionSetters();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
        ARM.setDefaultCommand(ARM.getGoToStateCommand(ArmStates.CLOSED).ignoringDisable(false));
        GRIPPER.setDefaultCommand(GRIPPER.getHoldCommand());
        leds.setDefaultCommand(flamesLEDCommand);
    }

    private void configureTargetPlacingPositionSetters() {
        OperatorConstants.LEVEL_1_TRIGGER.onTrue(new InstantCommand(() -> level.set(AllianceUtilities.isBlueAlliance() ? 1 : 3)).ignoringDisable(true));
        OperatorConstants.LEVEL_2_TRIGGER.onTrue(new InstantCommand(() -> level.set(2)).ignoringDisable(true));
        OperatorConstants.LEVEL_3_TRIGGER.onTrue(new InstantCommand(() -> level.set(AllianceUtilities.isBlueAlliance() ? 3 : 1)).ignoringDisable(true));

        staticPurpleColorLedCommand.setName("pur");
        staticYellowColorLedCommand.setName("yel");

        OperatorConstants.CONE_TRIGGER.onTrue(new InstantCommand(() -> {
            isCone.set(true);
            staticYellowColorLedCommand.schedule();
        }).ignoringDisable(true));
        OperatorConstants.CUBE_TRIGGER.onTrue(new InstantCommand(() -> {
            isCone.set(false);
            staticPurpleColorLedCommand.schedule();
        }).ignoringDisable(true));

        OperatorConstants.GRID_1_TRIGGER.onTrue(new InstantCommand(() -> grid.set(AllianceUtilities.isBlueAlliance() ? 1 : 3)).ignoringDisable(true));
        OperatorConstants.GRID_2_TRIGGER.onTrue(new InstantCommand(() -> grid.set(2)).ignoringDisable(true));
        OperatorConstants.GRID_3_TRIGGER.onTrue(new InstantCommand(() -> grid.set(AllianceUtilities.isBlueAlliance() ? 3 :  1)).ignoringDisable(true));

        OperatorConstants.LEFT_RAMP_TRIGGER.onTrue(new InstantCommand(() -> isLeftRamp.set(AllianceUtilities.isBlueAlliance())).ignoringDisable(true));
        OperatorConstants.RIGHT_RAMP_TRIGGER.onTrue(new InstantCommand(() -> isLeftRamp.set(!AllianceUtilities.isBlueAlliance())).ignoringDisable(true));
    }

    private void configureAutonomousChooser() {
        autonomousPathNameChooser.setDefaultOption("None", null);

        for (String currentPathName : AutonomousConstants.AUTONOMOUS_PATHS_NAMES)
            autonomousPathNameChooser.addOption(currentPathName, currentPathName);
    }

    private double calculateShiftModeValue() {
        final double squaredShiftModeValue = Math.pow(driverController.getRightTriggerAxis(), 2);

        return 1 - squaredShiftModeValue * OperatorConstants.MINIMUM_SHIFT_VALUE_COEFFICIENT;
    }

    @Log(name = "stickDegrees", methodName = "getDegrees")
    private Rotation2d getRightStickAsRotation2d() {
        if (isRightStickStill())
            return SWERVE.getHeading();

        return snapToClosest45Degrees(new Rotation2d(driverController.getRightY(), driverController.getRightX()));
    }

    private Rotation2d snapToClosest45Degrees(Rotation2d rotation2d) {
        return Rotation2d.fromDegrees(Math.round(rotation2d.getDegrees() / 45) * 45);
    }

    private boolean isRightStickStill() {
        return Math.abs(driverController.getRightY()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND &&
                Math.abs(driverController.getRightX()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND;
    }

    private void setPoseEstimatorPoseSources() {
        POSE_ESTIMATOR.addRobotPoseSources(CameraConstants.FORWARD_LIMELIGHT);
    }

    private FieldConstants.GridAlignment getGridAlignment() {
        if (!isCone.get())
            return FieldConstants.GridAlignment.getGridAlignment(grid.get(), 2);

        var res = FieldConstants.GridAlignment.getGridAlignment(
                grid.get(),
                isLeftRamp.get() ? 1 : 3
        );
        PoseEstimator.getInstance().getField().getObject("targetP").setPose(res.inFrontOfGridPose);
        SmartDashboard.putString("targetP", res.name());
        return FieldConstants.GridAlignment.getGridAlignment(
                grid.get(),
                isLeftRamp.get() ? 1 : 3
        );
    }

    private Pose2d setRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(
                pose.getX(),
                pose.getY(),
                rotation
        );
    }

    private ProxyCommand getGoToCurrentFirstArmPositionCommand() {
        return new ProxyCommand(() -> {
            if (isCone.get())
                return getGoToFirstConePositionCommand();

            return getGoToCurrentCubePositionCommand();
        });
    }

    private ProxyCommand getGoToCurrentSecondArmPositionCommand() {
        return new ProxyCommand(() -> {
            if (isCone.get())
                return getGoToSecondConePositionCommand();

            return getGoToCurrentCubePositionCommand();
        });
    }

    private CommandBase getGoToSecondConePositionCommand() {
        if (level.get() == 1)
            return ARM.getGoToStateCommand(ArmStates.HYBRID_1);
        if (level.get() == 2)
            return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_2);
        if (level.get() == 3)
            return ARM.getGoToStateCommand(ArmStates.CONE_HIGH_2);

        return new InstantCommand();
    }

    private CommandBase getGoToFirstConePositionCommand() {
        if (level.get() == 1)
            return ARM.getGoToStateCommand(ArmStates.HYBRID_1);
        if (level.get() == 2)
            return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_1);
        if (level.get() == 3)
            return ARM.getGoToStateCommand(ArmStates.CONE_HIGH_1);

        return new InstantCommand();
    }

    private CommandBase getGoToCurrentCubePositionCommand() {
        if (level.get() == 1)
            return ARM.getGoToStateCommand(ArmStates.HYBRID_1);
        if (level.get() == 2)
            return ARM.getGoToStateCommand(ArmStates.CUBE_MIDDLE_1);
        if (level.get() == 3)
            return ARM.getGoToStateCommand(ArmStates.CUBE_HIGH_1);

        return new InstantCommand();
    }

    private void toggleFieldAndSelfDrivenAngle() {
        if (SWERVE.getDefaultCommand().equals(fieldRelativeDriveFromSticksCommand)) {
            SWERVE.setDefaultCommand(fieldRelativeDrivenAngleFromSticksCommand);
            fieldRelativeDrivenAngleFromSticksCommand.schedule();
        } else {
            SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
            fieldRelativeDriveFromSticksCommand.schedule();
        }
    }

    private void setupArmBrakeModeWithUserButtonCommands() {
        userButton.onTrue(new InstantCommand(() -> ARM.setNeutralMode(false)).ignoringDisable(true));
        userButton.onFalse(new InstantCommand(ARM::setNeutralMode).ignoringDisable(true));
    }
}

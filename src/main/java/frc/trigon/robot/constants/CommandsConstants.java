package frc.trigon.robot.constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.components.cameras.collectioncamera.CollectionCamera;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.leds.commands.MovingColorsLedCommand;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.AllianceUtilities;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class CommandsConstants {
    private static final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();
    private static final CollectionCamera COLLECTION_CAMERA = RobotContainer.COLLECTION_CAMERA;
    private static final Swerve SWERVE = Swerve.getInstance();
    private static final Arm ARM = Arm.getInstance();
    private static final Gripper GRIPPER = Gripper.getInstance();
    private static final LedStrip LEDS = RobotContainer.LEDS;
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVE_CONTROLLER;
    private static final AtomicReference<Integer>
            LEVEL = new AtomicReference<>(1),
            GRID = new AtomicReference<>(1);
    private static final AtomicReference<Boolean>
            IS_CONE = new AtomicReference<>(false),
            IS_LEFT_RAMP = new AtomicReference<>(false);

    public static final CommandBase
            FIELD_RELATIVE_OPEN_LOOP_SUPPLIER_DRIVE_COMMAND = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue()
            ),
            SELF_RELATIVE_DRIVE_FROM_STICKS_COMMAND = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue()
            ).withName("selfRelative"),
            TURN_TO_FEEDER_WITH_MANUAL_DRIVE_COMMAND = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> Rotation2d.fromDegrees(AllianceUtilities.isBlueAlliance() ? 90 : -90)
            ),
            COLLECT_FROM_FEEDER_WITH_MANUAL_DRIVE_COMMAND = new ParallelCommandGroup(
                    TURN_TO_FEEDER_WITH_MANUAL_DRIVE_COMMAND,
                    GRIPPER.getSlowCollectCommand(),
                    ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_FEEDER, true, 0.5, 0.5)
            ),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(DRIVER_CONTROLLER.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> Math.sin(Units.degreesToRadians(-DRIVER_CONTROLLER.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> 0
            ),
            RESET_HEADING_COMMAND = new InstantCommand(
//                    () -> POSE_ESTIMATOR.resetPose(setRotation(POSE_ESTIMATOR.getCurrentPose(), new Rotation2d()))
                    () -> POSE_ESTIMATOR.resetPose(new Pose2d(5, 5, new Rotation2d()))
            ),
            TOGGLE_FIELD_AND_SELF_DRIVEN_COMMAND = new InstantCommand(
                    CommandsConstants::toggleFieldAndSelfDrivenAngle
            ),
            FIELD_RELATIVE_DRIVEN_ANGLE_FROM_STICKS_COMMAND = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> Rotation2d.fromDegrees(180)
            ),
            ALIGN_TO_GRID_COMMAND = Commands.getDriveToPoseCommand(
                    new PathConstraints(1, 1),
                    () -> getGridAlignment().inFrontOfGridPose
            ),
            APPLY_FIRST_ARM_STATE_COMMAND = Commands.getGoToCurrentFirstArmPositionCommand(IS_CONE::get, LEVEL::get),
            APPLY_SECOND_ARM_STATE_COMMAND = Commands.getGoToCurrentSecondArmPositionCommand(IS_CONE::get, LEVEL::get),
            RED_CLIMBING_LED_COMMAND = new MovingColorsLedCommand(LEDS, Color.kRed, 0.02, 5, Color.kBlack),
            FLAMES_LED_COMMAND = new MovingColorsLedCommand(LEDS, new Color(0f, 0f, 1f), 0.04, 7, Color.kRed),
            STATIC_YELLOW_COLOR_LED_COMMAND = new MovingColorsLedCommand(LEDS, Color.kBlack, 1, 0, Color.kYellow).withName("yellow"),
            STATIC_ORANGE_COLOR_LED_COMMAND = new MovingColorsLedCommand(LEDS, Color.kBlack, 1, 0, new Color(1, 0.1, 0)),
            STATIC_PURPLE_COLOR_LED_COMMAND = new MovingColorsLedCommand(LEDS, Color.kBlack, 1, 0, Color.kPurple).withName("purple"),
            RESET_POSE_TO_LIMELIGHT_POSE_COMMAND = new InstantCommand(
                    () -> POSE_ESTIMATOR.resetPose(CameraConstants.FORWARD_LIMELIGHT.getRobotPose())
            ).ignoringDisable(true),
            PRELOAD_CURRENT_AUTO_COMMAND = new InstantCommand(
                    CommandsConstants::preloadCurrentAuto
            ).ignoringDisable(true),
            DRIVE_AND_PLACE_COMMAND = Commands.getDriveAndPlaceCommand(CommandsConstants::getAlignmentPose, IS_CONE::get, LEVEL::get),
            CLOSED_COLLECTING_COMMAND = GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED_COLLECTING, true, 2, 1.7)),
            CLOSED_COLLECTING_STANDING_CONE_COMMAND = GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED_COLLECTING_STANDING_CONE, true, 2, 0.7)),
            RESET_POSE_TO_BEFORE_CHARGE_STATION_COMMAND = new InstantCommand(
                    ()-> POSE_ESTIMATOR.resetPose(new Pose2d(new Translation2d(2.22, 2.76), Rotation2d.fromDegrees(0)))
            ).ignoringDisable(true),
            RESET_POSE_TO_AFTER_CHARGE_STATION_COMMAND = new InstantCommand(
                    ()-> POSE_ESTIMATOR.resetPose(new Pose2d(new Translation2d(5.6, 2.76), Rotation2d.fromDegrees(0)))
            ).ignoringDisable(true),
            CLOSE_ARM_COMMAND = ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED, true, 1.4, 1.4).ignoringDisable(false),
            SET_LEVEL_TO_ONE_COMMAND = new InstantCommand(() -> LEVEL.set(1)).ignoringDisable(true),
            SET_LEVEL_TO_TWO_COMMAND = new InstantCommand(() -> LEVEL.set(2)).ignoringDisable(true),
            SET_LEVEL_TO_THREE_COMMAND = new InstantCommand(() -> LEVEL.set(3)).ignoringDisable(true),
            SET_TO_MIDDLE_CONE_COMMAND = new InstantCommand(() -> {
                        IS_CONE.set(true);
                        LEVEL.set(2);
                        STATIC_ORANGE_COLOR_LED_COMMAND.schedule();
            }).ignoringDisable(true),
            SET_TO_HIGH_CONE_COMMAND = new InstantCommand(() -> {
                        IS_CONE.set(true);
                        LEVEL.set(3);
                        STATIC_YELLOW_COLOR_LED_COMMAND.schedule();
            }).ignoringDisable(true),
            SET_TO_CUBE_COMMAND = new InstantCommand(() -> {
                        IS_CONE.set(false);
                        STATIC_PURPLE_COLOR_LED_COMMAND.schedule();
            }).ignoringDisable(true),
            SET_TO_GRID_1_COMMAND = new InstantCommand(() -> GRID.set(AllianceUtilities.isBlueAlliance() ? 1 : 3)).ignoringDisable(true),
            SET_TO_GRID_2_COMMAND = new InstantCommand(() -> GRID.set(2)).ignoringDisable(true),
            SET_TO_GRID_3_COMMAND = new InstantCommand(() -> GRID.set(AllianceUtilities.isBlueAlliance() ? 3 : 1)).ignoringDisable(true),
            SET_TO_LEFT_RAMP_COMMAND = new InstantCommand(() -> IS_LEFT_RAMP.set(AllianceUtilities.isBlueAlliance())).ignoringDisable(true),
            SET_TO_RIGHT_RAMP_COMMAND = new InstantCommand(() -> IS_LEFT_RAMP.set(!AllianceUtilities.isBlueAlliance())).ignoringDisable(true),
            GO_TO_TARGET_DASHBOARD_POSITION_COMMAND = new ProxyCommand(() ->
                    ARM.getGoToPositionCommand(SmartDashboard.getNumber("target1", 0), SmartDashboard.getNumber("target2", 0)
            ).ignoringDisable(true)),
            ARM_COAST_COMMAND = new InstantCommand(() -> ARM.setNeutralMode(false)).ignoringDisable(true),
            ARM_BRAKE_COMMAND = new InstantCommand(() -> ARM.setNeutralMode(true)).ignoringDisable(true),
            ARM_COAST_WITH_PINK_LED_COMMAND = new StartEndCommand(
                    () -> ARM.setNeutralMode(false),
                    () -> ARM.setNeutralMode(true)
            ).deadlineWith(Commands.fakeStaticColor(Color.kLightPink)).ignoringDisable(true);

    private static void preloadCurrentAuto() {
        final String pathName = RobotContainer.AUTONOMOUS_PATH_NAME_CHOOSER.get();
        if (pathName == null)
            return;
        final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, AutonomousConstants.AUTONOMOUS_PATH_CONSTRAINS);
        final Pose2d initialPose = pathGroup.get(0).getInitialHolonomicPose();
        final Pose2d initialAlliancePose = new Pose2d(
                initialPose.getX(),
                AllianceUtilities.isBlueAlliance() ? initialPose.getY() : FieldConstants.FIELD_WIDTH_METERS - initialPose.getY(),
                initialPose.getRotation()
        );

        POSE_ESTIMATOR.resetPose(initialAlliancePose);
        AutonomousConstants.PRELOADED_PATHS.put(RobotContainer.AUTONOMOUS_PATH_NAME_CHOOSER.get(), pathGroup);
    }

    private static Pose2d setRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(
                pose.getX(),
                pose.getY(),
                rotation
        );
    }

    private static Pose2d getAlignmentPose() {
        final Pose2d alignmentPose = getGridAlignment().inFrontOfGridPose;

        return new Pose2d(
                alignmentPose.getX(),
                alignmentPose.getY() + ((COLLECTION_CAMERA.getGamePiecePosition() / 100d * 1.25) * (DriverStation.getAlliance() == DriverStation.Alliance.Red ? -1 : 1)),
                alignmentPose.getRotation()
        );
    }

    private static FieldConstants.GridAlignment getGridAlignment() {
        if (!IS_CONE.get())
            return FieldConstants.GridAlignment.getGridAlignment(GRID.get(), 2);

        final FieldConstants.GridAlignment gridAlignment = FieldConstants.GridAlignment.getGridAlignment(
                GRID.get(),
                IS_LEFT_RAMP.get() ? 1 : 3
        );
        SmartDashboard.putString("targetGridAlignment", gridAlignment.name());
        return FieldConstants.GridAlignment.getGridAlignment(
                GRID.get(),
                IS_LEFT_RAMP.get() ? 1 : 3
        );
    }

    private static void toggleFieldAndSelfDrivenAngle() {
        if (SWERVE.getDefaultCommand().equals(SELF_RELATIVE_DRIVE_FROM_STICKS_COMMAND)) {
            SWERVE.setDefaultCommand(FIELD_RELATIVE_DRIVEN_ANGLE_FROM_STICKS_COMMAND);
            FIELD_RELATIVE_DRIVEN_ANGLE_FROM_STICKS_COMMAND.schedule();
        } else {
            SWERVE.setDefaultCommand(SELF_RELATIVE_DRIVE_FROM_STICKS_COMMAND);
            SELF_RELATIVE_DRIVE_FROM_STICKS_COMMAND.schedule();
        }
    }

    private static double calculateShiftModeValue() {
        final double squaredShiftModeValue = Math.pow(DRIVER_CONTROLLER.getRightTriggerAxis(), 2);

        return 1 - squaredShiftModeValue * OperatorConstants.MINIMUM_SHIFT_VALUE_COEFFICIENT;
    }
}

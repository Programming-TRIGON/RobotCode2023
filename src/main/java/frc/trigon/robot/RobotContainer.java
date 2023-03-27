package frc.trigon.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.AlignToReflectorCommand;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.CollectionCamera;
import frc.trigon.robot.components.ReflectionLimelight;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.leds.commands.MovingColorsLedCommand;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.KeyboardController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static frc.trigon.robot.subsystems.arm.ArmConstants.ArmStates;

public class RobotContainer implements Loggable {
    public static final ReflectionLimelight REFLECTION_LIMELIGHT = new ReflectionLimelight("limelight");
    // Subsystems TODO: make them not singletons

    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    public static final Arm ARM = Arm.getInstance();

    public static final Gripper GRIPPER = Gripper.getInstance();

    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
    private final CollectionCamera collectionCamera = new CollectionCamera("limelight-collection");

    // private final CollectionCamera COLLECTION_CAM = new CollectionCamera("limelight-collection");

    @Log(name = "autoChooser")
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();

    // Triggers
    private final XboxController driverController = OperatorConstants.DRIVE_CONTROLLER;
    private final KeyboardController keyboardController = OperatorConstants.KEYBOARD_CONTROLLER;
    private final Trigger userButton = new Trigger(() -> RobotController.getUserButton() || keyboardController.g().getAsBoolean());

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
            selfRelativeDriveFromSticksCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> driverController.getRightX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue()
            ).withName("selfRelative"),
            collectFromFeederWithManualDriveCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> Rotation2d.fromDegrees(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? 90 : -90)
            ),
            selfRelativeDriveFromDpadCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> Math.sin(Units.degreesToRadians(-driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> 0
            ),
            resetHeadingCommand = new InstantCommand(
                    () -> poseEstimator.resetPose(setRotation(poseEstimator.getCurrentPose(), new Rotation2d()))
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDrivenAngle
            ),
            fieldRelativeDrivenAngleFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> Rotation2d.fromDegrees(180)
            ),
            alignToGridCommand = Commands.getDriveToPoseCommand(
                    new PathConstraints(1, 1),
                    () -> getGridAlignment().inFrontOfGridPose
            ),
            applyFirstArmStateCommand = getGoToCurrentFirstArmPositionCommand(),
            applySecondArmStateCommand = getGoToCurrentSecondArmPositionCommand(),
            redClimbingLEDCommand = new MovingColorsLedCommand(leds, Color.kRed, 0.02, 5, Color.kBlack),
            flamesLEDCommand = new MovingColorsLedCommand(leds, new Color(0f, 0f, 1f), 0.04, 7, Color.kRed),
            staticYellowColorLedCommand = new MovingColorsLedCommand(leds, Color.kBlack, 1, 0, Color.kYellow),
            staticOrangeColorLedCommand = new MovingColorsLedCommand(leds, Color.kBlack, 1, 0, new Color(1, 0.1, 0)),
            staticPurpleColorLedCommand = new MovingColorsLedCommand(leds, Color.kBlack, 1, 0, Color.kPurple),
            resetPoseToLimelightPoseCommand = new InstantCommand(
                    () -> poseEstimator.resetPose(CameraConstants.FORWARD_LIMELIGHT.getRobotPose())
            ).ignoringDisable(true),
            preloadCurrentAutoCommand = new InstantCommand(
                    this::preloadCurrentAuto
            ).ignoringDisable(true);

    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        configureAutonomousChooser();
        setPoseEstimatorPoseSources();
        bindCommands();
        PhotonCamera.setVersionCheckEnabled(false);
        keyboardController.f8().whileTrue(Commands.getPlaceConeAtMidCommand());
        keyboardController.f9().whileTrue(Commands.getPlaceConeAtHighCommand());
        keyboardController.numpad0().whileTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));
        keyboardController.f7().whileTrue(new ProxyCommand(() -> Arm.getInstance().getGoToPositionCommand(SmartDashboard.getNumber("target1", 0), SmartDashboard.getNumber("target2", 0)).ignoringDisable(true)));

        SmartDashboard.putNumber("target1", SmartDashboard.getNumber("target1", 0));
        SmartDashboard.putNumber("target2", SmartDashboard.getNumber("target2", 0));

        configDriverCam();
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
        OperatorConstants.TURN_TO_GRID_TRIGGER.whileTrue(fieldRelativeDrivenAngleFromSticksCommand);
        //        OperatorConstants.LOCK_SWERVE_TRIGGER.whileTrue(SwerveCommands.getLockSwerveCommand());
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(selfRelativeDriveFromDpadCommand);
        OperatorConstants.ALIGN_TO_GRID_TRIGGER.whileTrue(alignToGridCommand);
        OperatorConstants.APPLY_FIRST_ARM_STATE_TRIGGER.whileTrue(applyFirstArmStateCommand);
        OperatorConstants.APPLY_SECOND_ARM_STATE_TRIGGER.whileTrue(applySecondArmStateCommand);
        OperatorConstants.EJECT_TRIGGER.whileTrue(Gripper.getInstance().getEjectCommand());
        OperatorConstants.START_AUTO_TRIGGER.whileTrue(new ProxyCommand(this::getAutonomousCommand));
        OperatorConstants.LED_FLAMES_TRIGGER.onTrue(flamesLEDCommand);
        OperatorConstants.PRELOAD_CURRENT_AUTO_TRIGGER.onTrue(preloadCurrentAutoCommand);

        OperatorConstants.GO_AND_PLACE_TRIGGER.whileTrue(getDriveAndPlaceCommand());
        OperatorConstants.SELF_RELATIVE_DRIVE_TRIGGER.onTrue(new InstantCommand(()->{
            if(selfRelativeDriveFromSticksCommand.isScheduled())
                selfRelativeDriveFromSticksCommand.cancel();
            else
                selfRelativeDriveFromSticksCommand.schedule();
        }));
        driverController.leftTrigger().whileTrue(GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CLOSED_COLLECTING, true, 2, 1.7)));
        driverController.rightBumper().whileTrue(GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CLOSED_COLLECTING_STANDING_CONE, true, 2, 0.7)));
        driverController.a().whileTrue(
                GRIPPER.getSlowCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CONE_FEEDER, true, 0.5, 0.5)).alongWith(
                        collectFromFeederWithManualDriveCommand));
        driverController.leftBumper().and(()->ARM.getDefaultCommand().equals(ARM.getCurrentCommand()) && ARM.atGoal()).whileTrue(
                new AlignToReflectorCommand()
        );

        OperatorConstants.BALANCE_TRIGGER.whileTrue(SwerveCommands.getBalanceCommand());
        configureTargetPlacingPositionSetters();
        keyboardController.f10().onTrue(new InstantCommand(
                ()-> poseEstimator.resetPose(new Pose2d(new Translation2d(2.22, 2.76), Rotation2d.fromDegrees(0)))
        ).ignoringDisable(true));

        keyboardController.f12().onTrue(new InstantCommand(
                ()-> poseEstimator.resetPose(new Pose2d(new Translation2d(5.6, 2.76), Rotation2d.fromDegrees(0)))
        ).ignoringDisable(true));
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
        ARM.setDefaultCommand(ARM.getGoToStateCommand(ArmStates.CLOSED, true, 1.4, 1.4).ignoringDisable(false));
        leds.setDefaultCommand(flamesLEDCommand);
    }

    private void configureTargetPlacingPositionSetters() {
        OperatorConstants.LEVEL_1_TRIGGER.onTrue(new InstantCommand(() -> level.set(1)).ignoringDisable(true));
        OperatorConstants.LEVEL_2_TRIGGER.onTrue(new InstantCommand(() -> level.set(2)).ignoringDisable(true));
        OperatorConstants.LEVEL_3_TRIGGER.onTrue(new InstantCommand(() -> level.set(3)).ignoringDisable(true));

        staticPurpleColorLedCommand.setName("pur");
        staticYellowColorLedCommand.setName("yel");

        OperatorConstants.CONE_TRIGGER.onTrue(new InstantCommand(() -> {
            isCone.set(true);
            level.set(2);
            staticOrangeColorLedCommand.schedule();
        }).ignoringDisable(true));
        OperatorConstants.HIGH_CONE_TRIGGER.onTrue(new InstantCommand(() -> {
            isCone.set(true);
            level.set(3);
            staticYellowColorLedCommand.schedule();
        }).ignoringDisable(true));
        OperatorConstants.CUBE_TRIGGER.onTrue(new InstantCommand(() -> {
            isCone.set(false);
            staticPurpleColorLedCommand.schedule();

        }).ignoringDisable(true));

        OperatorConstants.GRID_1_TRIGGER.onTrue(new InstantCommand(() -> grid.set(AllianceUtilities.isBlueAlliance() ? 1 : 3)).ignoringDisable(true));
        OperatorConstants.GRID_2_TRIGGER.onTrue(new InstantCommand(() -> grid.set(2)).ignoringDisable(true));
        OperatorConstants.GRID_3_TRIGGER.onTrue(new InstantCommand(() -> grid.set(AllianceUtilities.isBlueAlliance() ? 3 : 1)).ignoringDisable(true));

        OperatorConstants.LEFT_RAMP_TRIGGER.onTrue(new InstantCommand(() -> isLeftRamp.set(AllianceUtilities.isBlueAlliance())).ignoringDisable(true));
        OperatorConstants.RIGHT_RAMP_TRIGGER.onTrue(new InstantCommand(() -> isLeftRamp.set(!AllianceUtilities.isBlueAlliance())).ignoringDisable(true));

        OperatorConstants.CUBE_LOW_TRIGGER.whileTrue(Commands.getPlaceCubeAtHybridCommand());
        OperatorConstants.CUBE_MIDDLE_TRIGGER.whileTrue(Commands.getPlaceCubeAtMidCommand());
        OperatorConstants.CUBE_HIGH_TRIGGER.whileTrue(Commands.getPlaceCubeAtHighCommand());

        OperatorConstants.CONE_LOW_TRIGGER.whileTrue(Commands.getPlaceConeAtHybridCommand());
        OperatorConstants.CONE_MIDDLE_TRIGGER.whileTrue(Commands.getPlaceConeAtMidCommand());
        OperatorConstants.CONE_HIGH_TRIGGER.whileTrue(Commands.getPlaceConeAtHighCommand());

        keyboardController.a().whileTrue(new AlignToReflectorCommand());
        keyboardController.backtick().whileTrue(GRIPPER.getFullEjectCommand());
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

    private Rotation2d getRightStickAsRotation2d() {
        if (isRightStickStill())
            return poseEstimator.getCurrentPose().getRotation();

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
        poseEstimator.addRobotPoseSources(CameraConstants.FORWARD_LIMELIGHT);//,CameraConstants.t265);
    }

    private FieldConstants.GridAlignment getGridAlignment() {
        if (!isCone.get())
            return FieldConstants.GridAlignment.getGridAlignment(grid.get(), 2);

        final FieldConstants.GridAlignment gridAlignment = FieldConstants.GridAlignment.getGridAlignment(
                grid.get(),
                isLeftRamp.get() ? 1 : 3
        );
        SmartDashboard.putString("targetGridAlignment", gridAlignment.name());
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
            return ARM.getGoToStateCommand(ArmStates.CONE_HYBRID);
        if (level.get() == 2)
            return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_2);
        if (level.get() == 3)
            return ARM.getGoToStateCommand(ArmStates.CONE_HIGH);

        return new InstantCommand();
    }

    private CommandBase getGoToFirstConePositionCommand() {
        if (level.get() == 1)
            return ARM.getGoToStateCommand(ArmStates.CONE_HYBRID);
        if (level.get() == 2)
            return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_1);
        if (level.get() == 3)
            return ARM.getGoToStateCommand(ArmStates.CONE_HIGH);

        return new InstantCommand();
    }

    private CommandBase getGoToCurrentCubePositionCommand() {
        if (level.get() == 1)
            return ARM.getGoToStateCommand(ArmStates.CUBE_HYBRID);
        if (level.get() == 2)
            return ARM.getGoToStateCommand(ArmStates.CUBE_MIDDLE);
        if (level.get() == 3)
            return ARM.getGoToStateCommand(ArmStates.CUBE_HIGH);

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
        userButton.whileTrue(
                new StartEndCommand(
                        () -> ARM.setNeutralMode(false),
                        () -> ARM.setNeutralMode(true)
                ).deadlineWith(Commands.fakeStaticColor(Color.kLightPink)).ignoringDisable(true)
        );
    }

    private void configDriverCam() {
        var cam = CameraServer.startAutomaticCapture(2);
        cam.setResolution(424, 240);
        System.out.println(cam.setFPS(60));
        cam.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    }

    private CommandBase getDriveAndPlaceCommand() {
        return new ProxyCommand(Commands.getDriveToPoseCommand(
                new PathConstraints(1, 1),
                this::getAlignmentPose
        ).raceWith(Commands.fakeStaticColor(Color.kYellow)).andThen(
                new ProxyCommand(getPlaceCommand(isCone.get(), level.get()))
        ));
    }

    private Pose2d getAlignmentPose() {
        var alignment = getGridAlignment().inFrontOfGridPose;
        return new Pose2d(
                alignment.getX(),
                alignment.getY() + ((collectionCamera.getGamePiecePosition() / 100d * 1.25) * (DriverStation.getAlliance() == DriverStation.Alliance.Red ? -1 : 1)),
                alignment.getRotation()
        );
    }

    private void preloadCurrentAuto() {
        final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autonomousPathNameChooser.getSelected(), AutonomousConstants.AUTONOMOUS_PATH_CONSTRAINS);
        var ppjraj  = PathPlannerTrajectory.transformTrajectoryForAlliance(pathGroup.get(0), DriverStation.getAlliance());
        poseEstimator.resetPose(ppjraj.getInitialHolonomicPose());
        AutonomousConstants.PRELOADED_PATHS.put(autonomousPathNameChooser.getSelected(), pathGroup);
    }

    private Pose2d getHolonomicPose(PathPlannerTrajectory.PathPlannerState state) {
        return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
    }

    private CommandBase getPlaceCommand(boolean isCone, int level) {
        if(isCone) {
            if(level == 1)
                return Commands.getPlaceConeAtHybridCommand().withName("getPlaceConeAtHybridCommand");
            if(level == 2)
                return Commands.getPlaceConeAtMidCommand().withName("getPlaceConeAtMidCommand");
            if(level == 3)
                return Commands.getPlaceConeAtHighCommand().withName("getPlaceConeAtHighCommand");
        } else {
            if(level == 1)
                return Commands.getPlaceCubeAtHybridCommand().withName("getPlaceCubeAtHybridCommand");
            if(level == 2)
                return Commands.getPlaceCubeAtMidForAutoCommand().withName("getPlaceCubeAtMidCommand");
            if(level == 3)
                return Commands.getPlaceCubeAtHighForAutoCommand().withName("getPlaceCubeAtHighCommand");
        }
        return new InstantCommand();
    }

    public void teleopInit() {
        SWERVE.getDefaultCommand().schedule();
    }
}

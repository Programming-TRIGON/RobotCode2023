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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.leds.MasterLed;
import frc.trigon.robot.subsystems.leds.commands.MovingColorsLEDCommand;
import frc.trigon.robot.subsystems.leds.commands.StaticColorLEDCommand;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;

import java.util.concurrent.atomic.AtomicReference;

import static frc.trigon.robot.subsystems.arm.ArmConstants.ArmStates;

public class RobotContainer {
    @Log
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    public static final Arm ARM = Arm.getInstance();
    public static final Gripper GRIPPER = Gripper.getInstance();
    private final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();

//    private final CollectionCamera COLLECTION_CAM = new CollectionCamera("limelight-collection");

    @Log(name = "autoChooser")
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();
    private final MasterLed masterLed = MasterLed.getInstance();
    private final XboxController driverController = OperatorConstants.DRIVE_CONTROLLER;
    private final CommandGenericHID input = OperatorConstants.KEYBOARD_INPUT;
    private final Trigger userButton = new Trigger(() -> RobotController.getUserButton() || input.button(6).getAsBoolean());
    private final Trigger tippingTrigger = new Trigger(() -> Math.abs(SWERVE.getPitch()) < -2);
    private final AtomicReference<Integer>
            level = new AtomicReference<>(1),
            grid = new AtomicReference<>(1);
    private final AtomicReference<Boolean>
            isCone = new AtomicReference<>(false),
            isLeftRamp = new AtomicReference<>(false);

    private final LedStrip
            frontLeftLedStrip = new LedStrip(158, 33, true),
            frontRightLedStrip = new LedStrip(62, 33, true),
            rearLeftLedStrip = new LedStrip(0, 62, false),
            rearRightLedStrip = new LedStrip(95, 62, false);
    private final CommandBase
            fieldRelativeDriveFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / OperatorConstants.STICKS_DIVIDER / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / OperatorConstants.STICKS_DIVIDER/ calculateShiftModeValue(),
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
            redClimbingLEDCommand = new ParallelCommandGroup(
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, frontLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, frontRightLedStrip),
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, rearLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, rearRightLedStrip)
            ),
            flamesLEDCommand = new ParallelCommandGroup(
                    new MovingColorsLEDCommand(Color.kRed, new Color(1.0f, 0.3f, 0.0f), 0.02, 5, frontLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kRed, new Color(1.0f, 0.3f, 0.0f), 0.02, 5, frontRightLedStrip),
                    new MovingColorsLEDCommand(Color.kRed, new Color(1.0f, 0.3f, 0.0f), 0.02, 5, rearLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kRed, new Color(1.0f, 0.3f, 0.0f), 0.02, 5, rearRightLedStrip)
            ),
            staticYellowColorLedCommand = new ParallelCommandGroup(
                    new StaticColorLEDCommand(frontLeftLedStrip, new Color[]{Color.kYellow}, new int[]{1}),
                    new StaticColorLEDCommand(frontRightLedStrip, new Color[]{Color.kYellow}, new int[]{1}),
                    new StaticColorLEDCommand(rearLeftLedStrip, new Color[]{Color.kYellow}, new int[]{1}),
                    new StaticColorLEDCommand(rearRightLedStrip, new Color[]{Color.kYellow}, new int[]{1})
            ),
            staticPurpleColorLedCommand = new ParallelCommandGroup(
                    new StaticColorLEDCommand(frontLeftLedStrip, new Color[]{Color.kPurple}, new int[]{1}),
                    new StaticColorLEDCommand(frontRightLedStrip, new Color[]{Color.kPurple}, new int[]{1}),
                    new StaticColorLEDCommand(rearLeftLedStrip, new Color[]{Color.kPurple}, new int[]{1}),
                    new StaticColorLEDCommand(rearRightLedStrip, new Color[]{Color.kPurple}, new int[]{1})
            );

    public RobotContainer() {
        configureAutonomousChooser();
        setPoseEstimatorPoseSources();
        bindCommands();
        PhotonCamera.setVersionCheckEnabled(false);

        setupArmBrakeModeWithUserButtonCommands();
        SmartDashboard.putData(Arm.getInstance());

        driverController.rightBumper().whileTrue(
                ArmCommands.getPlaceCubeAtMiddleNodeCommand()
        );

        input.button(11).whileTrue(new ProxyCommand(() -> Arm.getInstance().getGoToPositionCommand(SmartDashboard.getNumber("target1", 0), SmartDashboard.getNumber("target2", 0), false).ignoringDisable(true)));
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
        addLedRequirements();
        bindControllerCommands();
        bindDefaultCommands();
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
        OperatorConstants.LED_FLAMES_TRIGGER.onTrue(new InstantCommand(flamesLEDCommand::schedule));
        OperatorConstants.PLACE_GAME_PIECE_AT_HYBRID_TRIGGER.whileTrue(placeGamePieceAtHybridCommand);
        tippingTrigger.onTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));

        driverController.leftTrigger().whileTrue(GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CLOSED_COLLECTING, true)));
        driverController.leftBumper().whileTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));

        configureTargetPlacingPositionSetters();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
        ARM.setDefaultCommand(ARM.getGoToStateCommand(ArmStates.CLOSED).ignoringDisable(false));
        GRIPPER.setDefaultCommand(GRIPPER.getHoldCommand());
        masterLed.setDefaultCommand(flamesLEDCommand);
    }

    private void configureTargetPlacingPositionSetters() {
        OperatorConstants.LEVEL_1_TRIGGER.onTrue(new InstantCommand(() -> level.set(AllianceUtilities.isBlueAlliance() ? 1 : 3)).ignoringDisable(true));
        OperatorConstants.LEVEL_2_TRIGGER.onTrue(new InstantCommand(() -> level.set(2)).ignoringDisable(true));
        OperatorConstants.LEVEL_3_TRIGGER.onTrue(new InstantCommand(() -> level.set(AllianceUtilities.isBlueAlliance() ? 3 : 1)).ignoringDisable(true));

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

    private void addLedRequirements() {
        flamesLEDCommand.addRequirements(masterLed);
        staticYellowColorLedCommand.addRequirements(masterLed);
        staticPurpleColorLedCommand.addRequirements(masterLed);
        redClimbingLEDCommand.addRequirements(masterLed);
    }

    private boolean isRightStickStill() {
        return Math.abs(driverController.getRightY()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND &&
                Math.abs(driverController.getRightX()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND;
    }

    private void setPoseEstimatorPoseSources() {
//        POSE_ESTIMATOR.addRobotPoseSources(CameraConstants.FORWARD_LIMELIGHT);
    }

    private FieldConstants.GridAlignment getGridAlignment() {
        if (!isCone.get())
            return FieldConstants.GridAlignment.getGridAlignment(grid.get(), 2);

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
        final ProxyCommand applyFirstStateCommand = new ProxyCommand(() -> {
            if (!isCone.get())
                return getGoToFirstConePositionCommand();

            return getGoToCurrentCubePositionCommand();
        });
        applyFirstStateCommand.addRequirements(ARM);

        return applyFirstStateCommand;
    }

    private ProxyCommand getGoToCurrentSecondArmPositionCommand() {
        final ProxyCommand applySecondArmStateCommand = new ProxyCommand(() -> {
            if (isCone.get())
                return getGoToSecondConePositionCommand();

            return getGoToCurrentCubePositionCommand();
        });
        applySecondArmStateCommand.addRequirements(ARM);

        return applySecondArmStateCommand;
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

package frc.trigon.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.concurrent.atomic.AtomicReference;

import static frc.trigon.robot.subsystems.arm.ArmConstants.ArmStates;

public class RobotContainer implements Loggable {
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    public static final Arm ARM = Arm.getInstance();
    public static final Gripper GRIPPER = Gripper.getInstance();
    private final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();

//    private final CollectionCamera COLLECTION_CAM = new CollectionCamera("limelight-collection");

    @Log(name = "autoChooser")
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();

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
                    () -> SWERVE.setHeading(new Rotation2d())
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
                    () -> getGridAlignment().inFrontOfGridPose,
                    true
            ),
            applyFirstArmStateCommand = getApplyFirstArmStateCommand(),
            applySecondArmStateCommand = getApplySecondArmStateCommand();

    public RobotContainer() {
        for (FieldConstants.GridAlignment gridAlignment : FieldConstants.GridAlignment.values())
            PoseEstimator.getInstance().getField().getObject(gridAlignment.name()).setPose(gridAlignment.inFrontOfGridPose);
        configureAutonomousChooser();
        setPoseEstimatorPoseSources();
        bindCommands();

        setupArmBrakeModeWithUserButtonCommands();
        SmartDashboard.putData(Arm.getInstance());

        driverController.rightBumper().whileTrue(
                ArmCommands.getPlaceCubeAtMiddleNodeCommand()
        );
        var pose = new Pose2d(14.48, 1.6, new Rotation2d());
        driverController.leftBumper().whileTrue(
                Commands.getDriveToPoseCommand(
                                new PathConstraints(1, 1),
                                () -> pose,
                                true
                        )
                        .andThen(
                                ArmCommands.getPlaceConeAtMediumNodeCommand()
                        )
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
        tippingTrigger.onTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));

        driverController.leftTrigger().whileTrue(GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CLOSED_COLLECTING, true)));
        driverController.leftBumper().whileTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));

        configureTargetPlacingPositionSetters();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
        ARM.setDefaultCommand(ARM.getGoToStateCommand(ArmStates.CLOSED).ignoringDisable(false));
        GRIPPER.setDefaultCommand(GRIPPER.getHoldCommand());
    }

    private void configureTargetPlacingPositionSetters() {
        OperatorConstants.LEVEL_1_TRIGGER.onTrue(new InstantCommand(() -> level.set(1)).ignoringDisable(true));
        OperatorConstants.LEVEL_2_TRIGGER.onTrue(new InstantCommand(() -> level.set(2)).ignoringDisable(true));
        OperatorConstants.LEVEL_3_TRIGGER.onTrue(new InstantCommand(() -> level.set(3)).ignoringDisable(true));
        OperatorConstants.START_AUTO_TRIGGER.whileTrue(new ProxyCommand(this::getAutonomousCommand));

        OperatorConstants.CONE_TRIGGER.onTrue(new InstantCommand(() -> isCone.set(true)).ignoringDisable(true));
        OperatorConstants.CUBE_TRIGGER.onTrue(new InstantCommand(() -> isCone.set(false)).ignoringDisable(true));

        OperatorConstants.GRID_1_TRIGGER.onTrue(new InstantCommand(() -> grid.set(1)).ignoringDisable(true));
        OperatorConstants.GRID_2_TRIGGER.onTrue(new InstantCommand(() -> grid.set(2)).ignoringDisable(true));
        OperatorConstants.GRID_3_TRIGGER.onTrue(new InstantCommand(() -> grid.set(3)).ignoringDisable(true));

        OperatorConstants.LEFT_RAMP_TRIGGER.onTrue(new InstantCommand(() -> isLeftRamp.set(true)).ignoringDisable(true));
        OperatorConstants.RIGHT_RAMP_TRIGGER.onTrue(new InstantCommand(() -> isLeftRamp.set(false)).ignoringDisable(true));
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

        return FieldConstants.GridAlignment.getGridAlignment(
                grid.get(),
                isLeftRamp.get() ? 1 : 3
        );
    }

    private ProxyCommand getApplySecondArmStateCommand() {
        return new ProxyCommand(() -> {
            if (isCone.get()) {
                if (level.get() == 1)
                    return ARM.getGoToStateCommand(ArmStates.CONE_HYBRID_1);
                else if (level.get() == 2)
                    return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_2);
                else if (level.get() == 3)
                    return ARM.getGoToStateCommand(ArmStates.CONE_HIGH_2);
            }
            return getCubeArmToFirstLevelCommand();
        });
    }

    private ProxyCommand getApplyFirstArmStateCommand() {
        return new ProxyCommand(() -> {
            if (isCone.get()) {
                if (level.get() == 1)
                    return ARM.getGoToStateCommand(ArmStates.CONE_HYBRID_1).alongWith(GRIPPER.getStopCommand());
                else if (level.get() == 2)
                    return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_1).alongWith(GRIPPER.getStopCommand());
                else if (level.get() == 3)
                    return ARM.getGoToStateCommand(ArmStates.CONE_HIGH_1).alongWith(GRIPPER.getStopCommand());
            }
            return getCubeArmToFirstLevelCommand();
        });
    }

    private Command getCubeArmToFirstLevelCommand() {
        if (level.get() == 1)
            return ARM.getGoToStateCommand(ArmStates.CUBE_HYBRID_1);
        else if (level.get() == 2)
            return ARM.getGoToStateCommand(ArmStates.CUBE_MIDDLE_1);
        else if (level.get() == 3)
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

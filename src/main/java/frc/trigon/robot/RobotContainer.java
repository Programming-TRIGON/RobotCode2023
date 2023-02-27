package frc.trigon.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.CollectionCamera;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
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

    private final CollectionCamera COLLECTION_CAM = new CollectionCamera("limelight-collection");

    @Log(name = "autoChooser")
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();

    private final XboxController driverController = OperatorConstants.DRIVE_CONTROLLER;
    CommandGenericHID input = new CommandGenericHID(1);
    private final Trigger userButton = new Trigger(() -> RobotController.getUserButton() || input.button(6).getAsBoolean());
    private final Trigger tippingTrigger = new Trigger(() -> Math.abs(SWERVE.getPitch()) < -2);

    private final CommandBase
            fieldRelativeDriveFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / calculateShiftModeValue(),
                    () -> driverController.getRightX() / calculateShiftModeValue()
            ),
            selfRelativeDriveFromDpadCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> Math.sin(Units.degreesToRadians(-driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> 0
            ),
            resetPoseCommand = new InstantCommand(
                    () -> POSE_ESTIMATOR.resetPose(new Pose2d())
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDrivenAngle
            ),
            fieldRelativeDrivenAngleFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / calculateShiftModeValue(),
                    this::getRightStickAsRotation2d
            );

    public RobotContainer() {
        configureAutonomousChooser();
        setPoseEstimatorPoseSources();
        bindCommands();

        setupArmBrakeModeWithUserButtonCommands();
        SmartDashboard.putData(Arm.getInstance());

        driverController.leftTrigger().whileTrue(GRIPPER.getCollectCommand().alongWith(ARM.getGoToStateCommand(ArmStates.CLOSED_COLLECTING, true)));
        driverController.leftBumper().whileTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));
        AtomicReference<Integer> level = new AtomicReference<>(1);
        input.button(1).onTrue(new InstantCommand(() -> level.set(1)).ignoringDisable(true));
        input.button(2).onTrue(new InstantCommand(() -> level.set(2)).ignoringDisable(true));
        input.button(3).onTrue(new InstantCommand(() -> level.set(3)).ignoringDisable(true));

        AtomicReference<Boolean> isCone = new AtomicReference<>(false);
        input.button(4).onTrue(new InstantCommand(() -> isCone.set(true)).ignoringDisable(true));
        input.button(5).onTrue(new InstantCommand(() -> isCone.set(false)).ignoringDisable(true));

        input.button(6).whileTrue(new ProxyCommand(() -> {
            if(isCone.get()) {
                if(level.get() == 1) {
                    return ARM.getGoToStateCommand(ArmStates.CONE_HYBRID_1);
                } else if(level.get() == 2) {
                    return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_1);
                } else if(level.get() == 3) {
                    return ARM.getGoToStateCommand(ArmStates.CONE_HIGH_1);
                }
            }
            if(level.get() == 1) {
                return ARM.getGoToStateCommand(ArmStates.CUBE_HYBRID_1);
            } else if(level.get() == 2) {
                return ARM.getGoToStateCommand(ArmStates.CUBE_MIDDLE_1);
            } else if(level.get() == 3) {
                return ARM.getGoToStateCommand(ArmStates.CUBE_HIGH_1);
            }
            return new InstantCommand();
        }));
        input.button(7).whileTrue(new ProxyCommand(() -> {
            if(isCone.get()) {
                if(level.get() == 1)
                    return ARM.getGoToStateCommand(ArmStates.CONE_HYBRID_1);
                else if(level.get() == 2)
                    return ARM.getGoToStateCommand(ArmStates.CONE_MIDDLE_2);
                else if(level.get() == 3)
                    return ARM.getGoToStateCommand(ArmStates.CONE_HIGH_2);
            }
            if(level.get() == 1) {
                return ARM.getGoToStateCommand(ArmStates.CUBE_HYBRID_1);
            } else if(level.get() == 2) {
                return ARM.getGoToStateCommand(ArmStates.CUBE_MIDDLE_1);
            } else if(level.get() == 3) {
                return ARM.getGoToStateCommand(ArmStates.CUBE_HIGH_1);
            }
            return new InstantCommand();
        }));

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

        ARM.setDefaultCommand(ARM.getGoToStateCommand(ArmStates.CLOSED).ignoringDisable(false));

        input.button(8).whileTrue(Gripper.getInstance().getCollectCommand());
        input.button(9).whileTrue(Gripper.getInstance().getEjectCommand());
        input.button(10).whileTrue(Gripper.getInstance().getHoldCommand());

        input.button(11).whileTrue(new ProxyCommand(() -> Arm.getInstance().getGoToPositionCommand(SmartDashboard.getNumber("target1", 0), SmartDashboard.getNumber("target2", 0), false).ignoringDisable(true)));
        SmartDashboard.putNumber("target1", SmartDashboard.getNumber("target1", 0));
        SmartDashboard.putNumber("target2", SmartDashboard.getNumber("target2", 0));
    }

    /**
     * @return the command to run in autonomous mode
     */
    CommandBase getAutonomousCommand() {
        if (autonomousPathNameChooser.getSelected() == null)
            return null;

        return Commands.getAutonomousCommand(autonomousPathNameChooser.getSelected());
    }

    private void bindCommands() {
        bindControllerCommands();
        bindDefaultCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
        //        GRIPPER.setDefaultCommand(GRIPPER.getHoldCommand());
        tippingTrigger.onTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_POSE_TRIGGER.onTrue(resetPoseCommand);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_DRIVEN_ANGLE_TRIGGER.onTrue(toggleFieldAndSelfDrivenCommand);
        OperatorConstants.LOCK_SWERVE_TRIGGER.whileTrue(SwerveCommands.getLockSwerveCommand());
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(selfRelativeDriveFromDpadCommand);
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
        return Math.abs(driverController.getRightY()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND&&
                Math.abs(driverController.getRightX()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND;
    }

    private void setPoseEstimatorPoseSources() {
        POSE_ESTIMATOR.addRobotPoseSources(CameraConstants.FORWARD_LIMELIGHT);
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

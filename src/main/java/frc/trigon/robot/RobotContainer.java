package frc.trigon.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.CollectionCamera;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.subsystems.arm.Arm;
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
    private final Arm ARM = Arm.getInstance();
    private final Gripper GRIPPER = Gripper.getInstance();
    private final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();

    private final CollectionCamera COLLECTION_CAM = new CollectionCamera("limelight-collection");

    private final XboxController driverController = DriverConstants.DRIVE_CONTROLLER;
    CommandGenericHID input = new CommandGenericHID(1);
    private final Trigger userButton = new Trigger(() -> RobotController.getUserButton() || input.button(6).getAsBoolean());
    private final Trigger tippingTrigger = new Trigger(() -> Math.abs(SWERVE.getPitch()) < -2);

    private final CommandBase
            fieldRelativeDriveFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX
    ),
            selfRelativeDriveFromDpadCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> -Math.cos(Units.degreesToRadians(driverController.getPov())) / DriverConstants.POV_DIVIDER,
                    () -> Math.sin(Units.degreesToRadians(driverController.getPov())) / DriverConstants.POV_DIVIDER,
                    () -> 0
            ),
            resetPoseCommand = new InstantCommand(
                    () -> POSE_ESTIMATOR.resetPose(new Pose2d())
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDrivenAngle
            );

    public RobotContainer() {
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

        ARM.setDefaultCommand(ARM.getGoToStateCommand(ArmStates.CLOSED).ignoringDisable(false));

        input.button(8).whileTrue(Gripper.getInstance().getCollectCommand());
        input.button(9).whileTrue(Gripper.getInstance().getEjectCommand());
        input.button(10).whileTrue(Gripper.getInstance().getHoldCommand());

        input.button(11).whileTrue(new ProxyCommand(() -> Arm.getInstance().getGoToPositionCommand(SmartDashboard.getNumber("target1", 0), SmartDashboard.getNumber("target2", 0), false).ignoringDisable(true)));
        SmartDashboard.putNumber("target1", SmartDashboard.getNumber("target1", 0));
        SmartDashboard.putNumber("target2", SmartDashboard.getNumber("target2", 0));
    }

    private void bindCommands() {
        bindControllerCommands();
        bindDefaultCommands();
    }

    private void bindControllerCommands() {
        DriverConstants.RESET_POSE_TRIGGER.onTrue(resetPoseCommand);
        DriverConstants.TOGGLE_FIELD_AND_SELF_DRIVEN_ANGLE_TRIGGER.onTrue(toggleFieldAndSelfDrivenCommand);
        DriverConstants.LOCK_SWERVE_TRIGGER.whileTrue(SwerveCommands.getLockSwerveCommand());
        DriverConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(selfRelativeDriveFromDpadCommand);
    }

    @Log(methodName = "getDegrees")
    private Rotation2d getRightStickAsRotation2d() {
        return new Rotation2d(driverController.getRightX(), driverController.getRightY());
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
        GRIPPER.setDefaultCommand(GRIPPER.getHoldCommand());
        tippingTrigger.onTrue(ARM.getGoToStateCommand(ArmStates.CLOSED));
    }

    private void setPoseEstimatorPoseSources() {
        //        poseEstimator.addRobotPoseSources(forwardLimelight);
    }

    private void toggleFieldAndSelfDrivenAngle() {
        if(SWERVE.getDefaultCommand().equals(fieldRelativeDriveFromSticksCommand))
            SWERVE.setDefaultCommand(new StartEndCommand(() -> {}, () -> {}));
        else
            SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
    }

    private void setupArmBrakeModeWithUserButtonCommands() {
        userButton.onTrue(new InstantCommand(() -> ARM.setNeutralMode(false)).ignoringDisable(true));
        userButton.onFalse(new InstantCommand(ARM::setNeutralMode).ignoringDisable(true));
    }
}

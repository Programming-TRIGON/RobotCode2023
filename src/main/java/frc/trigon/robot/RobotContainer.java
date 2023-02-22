package frc.trigon.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.robotposesources.AprilTagPhotonCamera;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {
    @Log
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
    private final RobotPoseSource forwardLimelight = new AprilTagPhotonCamera(
            "limelight-forward",
            new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0)
            )
    );
    private final XboxController driverController = DriverConstants.DRIVE_CONTROLLER;

    private final Command
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
                    () -> poseEstimator.resetPose(new Pose2d())
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDrivenAngle
            );

    public RobotContainer() {
        setPoseEstimatorPoseSources();
        bindCommands();
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
    }

    private void setPoseEstimatorPoseSources() {
        poseEstimator.addRobotPoseSources(forwardLimelight);
    }

    private void toggleFieldAndSelfDrivenAngle() {
        if (SWERVE.getDefaultCommand().equals(fieldRelativeDriveFromSticksCommand))
            SWERVE.setDefaultCommand(new StartEndCommand(() -> {}, () -> {}));
        else
            SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
    }
}

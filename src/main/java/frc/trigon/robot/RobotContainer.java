package frc.trigon.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.robotposesources.AprilTagPhotonCamera;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;

public class RobotContainer {
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
    private final Trigger
            resetPoseTrigger = driverController.y(),
            toggleFieldAndSelfDrivenTrigger = driverController.x();

    private final Command
            fieldRelativeDriveFromControllerCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    driverController::getLeftY,
                    driverController::getLeftX,
                    driverController::getRightX
            ),
            selfRelativeDriveFromControllerCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    driverController::getLeftY,
                    driverController::getLeftX,
                    driverController::getRightX
            ),
            resetPoseCommand = new InstantCommand(
                    () -> poseEstimator.resetPose(new Pose2d())
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDriven
            );


    public RobotContainer() {
        setPoseEstimatorRobotPoseSources();
        bindCommands();
    }

    private void bindCommands() {
        bindControllerCommands();
        bindDefaultCommands();
    }

    private void bindControllerCommands() {
        resetPoseTrigger.onTrue(resetPoseCommand);
        toggleFieldAndSelfDrivenTrigger.onTrue(toggleFieldAndSelfDrivenCommand);
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromControllerCommand);
    }

    private void setPoseEstimatorRobotPoseSources() {
        poseEstimator.addRobotPoseSources(forwardLimelight);
    }

    private void toggleFieldAndSelfDriven() {
        if (SWERVE.getDefaultCommand().equals(fieldRelativeDriveFromControllerCommand))
            SWERVE.setDefaultCommand(selfRelativeDriveFromControllerCommand);
        else
            SWERVE.setDefaultCommand(fieldRelativeDriveFromControllerCommand);
    }
}

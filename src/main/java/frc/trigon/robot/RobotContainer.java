package frc.trigon.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.posesources.PhotonCameraPoseSource;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class RobotContainer {
    private static final PhotonCameraPoseSource CAMERA_POSE_SOURCE = new PhotonCameraPoseSource(
            "photonlimelight",
            new Transform3d()
    );
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(CAMERA_POSE_SOURCE);
    private final Swerve swerve = Swerve.getInstance();

    private final XboxController mainController = new XboxController(0);
    private final Command
            fieldRelativeDriveWithMainController = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> -mainController.getLeftY(),
                    () -> -mainController.getLeftX(),
                    () -> -mainController.getRightX()
            ),
            poseEstimatorUpdater = POSE_ESTIMATOR.getUpdatePoseEstimatorCommand(),
            resetOdometryCommand = new InstantCommand(
                    () -> POSE_ESTIMATOR.resetPose(new Pose2d())
            );

    private final Trigger resetOdometryTrigger = new Trigger(mainController::getYButton);

    public RobotContainer() {
        configureCommandBindings();
    }

    private void configureCommandBindings() {
        swerve.setDefaultCommand(fieldRelativeDriveWithMainController);
        POSE_ESTIMATOR.setDefaultCommand(poseEstimatorUpdater);

        resetOdometryTrigger.onTrue(resetOdometryCommand);
    }
}

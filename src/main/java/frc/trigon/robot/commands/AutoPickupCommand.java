package frc.trigon.robot.commands;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.trigon.robot.components.D415;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class AutoPickupCommand extends SequentialCommandGroup {
    private static final PathConstraints PATH_CONSTRAINS = AutonomousConstants.AUTONOMOUS_PATH_CONSTRAINS;
    private final D415 d415 = CameraConstants.D415;

    public AutoPickupCommand() {
        final CommandBase driveUntilPickupCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                () -> 0.5, () -> 0, () -> 0
        ).until(Gripper.getInstance()::isHolding);

        addCommands(
                new WaitUntilCommand(d415::hasVisibleGamePieces),
                Commands.getDriveToPoseCommand(PATH_CONSTRAINS, () -> calculatePickupPose(d415.getBestGamePiece())),
                CommandsConstants.CLOSED_COLLECTING_COMMAND,
                driveUntilPickupCommand
        );
    }

    private Pose2d calculatePickupPose(D415.GamePiece gamePiece) {
        final Pose2d
                currentPose = PoseEstimator.getInstance().getCurrentPose(),
                relativeGamePiecePose = gamePiece.getPose().toPose2d(),
                gamePiecePose = currentPose.transformBy(poseToTransform(relativeGamePiecePose));

        return gamePiecePose.transformBy(AutonomousConstants.GAME_PIECE_TO_PICKUP_POSE);
    }

    private Pose2d transformToPose(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    private Transform2d poseToTransform(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }
}

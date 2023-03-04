package frc.trigon.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.List;
import java.util.function.Supplier;

public class Commands {
    private static final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();

    /**
     * Creates a command that follows a path group from an autonomous path's name. This command will also reset the pose.
     *
     * @param pathName the autonomous path's name
     * @return the command
     */
    public static SequentialCommandGroup getAutonomousCommand(String pathName) {
        final List<PathPlannerTrajectory> autonomousPathGroup = PathPlanner.loadPathGroup(pathName, AutonomousConstants.AUTONOMOUS_PATH_CONSTRAINS);

        return SwerveCommands.getFollowPathGroupCommand(autonomousPathGroup, AutonomousConstants.EVENT_MAP);
    }

    /**
     * Creates a command that drives to a pose.
     *
     * @param driveConstraints   the drive constraints
     * @param targetPoseSupplier the target pose supplier
     * @return the command
     */
    public static ProxyCommand getDriveToPoseCommand(PathConstraints driveConstraints, Supplier<Pose2d> targetPoseSupplier) {
        return new ProxyCommand(() -> getDriveToPoseCommand(driveConstraints, targetPoseSupplier.get()));
    }

    private static CommandBase getDriveToPoseCommand(PathConstraints driveConstraints, Pose2d targetPose) {
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
        final PathPoint currentPoint = new PathPoint(
                currentPose.getTranslation(),
                currentPose.getRotation(),
                currentPose.getRotation()
        );
        final PathPoint targetPoint = new PathPoint(
                targetPose.getTranslation(),
                targetPose.getRotation(),
                targetPose.getRotation()
        );
        final PathPlannerTrajectory path = PathPlanner.generatePath(driveConstraints, currentPoint, targetPoint);

        return SwerveCommands.getFollowPathCommand(path).andThen(SwerveCommands.getDriveToPoseWithPIDCommand(targetPose));
    }
}

package frc.trigon.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
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

        return SwerveCommands.getFollowPathGroupCommand(autonomousPathGroup, AutonomousConstants.EVENT_MAP, true);
    }

    /**
     * Creates a command that drives to a pose.
     *
     * @param driveConstraints   the drive constraints
     * @param targetPoseSupplier the target pose supplier
     * @param useAllianceColor   whether to use the alliance color
     * @return the command
     */
    public static ProxyCommand getDriveToPoseCommand(PathConstraints driveConstraints, Supplier<Pose2d> targetPoseSupplier, boolean useAllianceColor) {
        final Supplier<Command> driveToPointCommandSupplier = () -> {
            final Pose2d
                    currentPose = POSE_ESTIMATOR.getCurrentPose(),
                    targetPose = useAllianceColor ? mirrorPoseRelativeToColor(targetPoseSupplier.get()) : targetPoseSupplier.get();
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
        };
        return new ProxyCommand(driveToPointCommandSupplier);
    }

    private static Pose2d mirrorPoseRelativeToColor(Pose2d pose) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
            return pose;

        return new Pose2d(
                FieldConstants.FIELD_LENGTH_METERS - pose.getX(),
                pose.getY(),
                mirrorRotation(pose.getRotation(), FieldConstants.FIELD_MIRRORING_LINE)
        );
    }

    private static Rotation2d mirrorRotation(Rotation2d rotation, Rotation2d mirroringLine) {
        return rotation.plus(mirroringLine).unaryMinus().minus(mirroringLine);
    }
}

package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class SwerveCommands {
    private static final Swerve SWERVE = Swerve.getInstance();

    /**
     * Creates a command that will drive the robot using the given path group and event map.
     *
     * @param pathGroup the path group to follow
     * @param eventMap the event map to use
     * @param useAllianceColor whether to use the alliance color
     * @return the command
     */
    public static Command getFollowPathGroupCommand(List<PathPlannerTrajectory> pathGroup, Map<String, Command> eventMap, boolean useAllianceColor) {
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                RobotContainer.POSE_ESTIMATOR::getCurrentPose,
                (pose2d) -> {},
                SwerveConstants.KINEMATICS,
                SwerveConstants.TRANSLATION_PID_CONSTANTS,
                SwerveConstants.ROTATION_PID_CONSTANTS,
                SWERVE::setTargetModuleStates,
                eventMap,
                useAllianceColor,
                SWERVE
        );

        return swerveAutoBuilder.fullAuto(pathGroup);
    }

    /**
     * Creates a command that will drive the robot using the given path and event map.
     *
     * @param path the path group to follow
     * @param useAllianceColor whether to use the alliance color
     * @return the command
     */
    public static Command getFollowPathCommand(PathPlannerTrajectory path, boolean useAllianceColor) {
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                RobotContainer.POSE_ESTIMATOR::getCurrentPose,
                (pose2d) -> {},
                SwerveConstants.KINEMATICS,
                SwerveConstants.TRANSLATION_PID_CONSTANTS,
                SwerveConstants.ROTATION_PID_CONSTANTS,
                SWERVE::setTargetModuleStates,
                new HashMap<>(),
                useAllianceColor,
                SWERVE
        );

        return swerveAutoBuilder.followPath(path);
    }

    /**
     * Creates a command that sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     * @return the command
     */
    public static CommandBase getSetSwerveBrakeCommand(boolean brake) {
        return new InstantCommand(() -> SWERVE.setBrake(brake), SWERVE);
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the field's frame of reference, in closed loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getFieldRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitializeRunnable(true),
                getFieldRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the robot's frame of reference, in closed loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getSelfRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitializeRunnable(true),
                getSelfRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the robot's frame of reference, in open loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getSelfRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitializeRunnable(false),
                getSelfRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the field's frame of reference, in open loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getFieldRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitializeRunnable(false),
                getFieldRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }

    private static Runnable getDriveInitializeRunnable(boolean closedLoop) {
        return () -> {
            SWERVE.setBrake(true);
            SWERVE.setClosedLoop(closedLoop);
        };
    }

    private static Runnable getFieldRelativeRunnable(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return () -> SWERVE.fieldRelativeDrive(
                new Translation2d(
                        x.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                        y.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                ),
                new Rotation2d(
                        theta.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
                )
        );
    }

    private static Runnable getSelfRelativeRunnable(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return () -> SWERVE.selfRelativeDrive(
                new Translation2d(
                        x.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                        y.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                ),
                new Rotation2d(
                        theta.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
                )
        );
    }

    private static Consumer<Boolean> getDriveStopConsumer() {
        return (interrupted) -> {
            SWERVE.stop();
            SWERVE.setBrake(false);
        };
    }

}

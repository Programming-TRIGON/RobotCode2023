package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    private static final Swerve SWERVE = RobotContainer.SWERVE;
    private static final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();

    /**
     * Creates a command that locks the swerve and brakes it.
     * This should be used when you want to make it hard to move the swerve.
     *
     * @return the command
     */
    public static StartEndCommand getLockSwerveCommand() {
        return new StartEndCommand(SWERVE::lockSwerve, () -> {}, SWERVE);
    }

    /**
     * @return a command that brakes the swerve modules and then coasts them, runs when disabled
     */
    public static WrapperCommand getBrakeAndCoastCommand() {
        return SwerveCommands.getSetSwerveBrakeCommand(true)
                .andThen(new WaitCommand(SWERVE.getBrakeTimeSeconds()))
                .andThen(SwerveCommands.getSetSwerveBrakeCommand(false))
                .ignoringDisable(true);
    }

    /**
     * Creates a command that will drive the robot using the given path group and event map.
     *
     * @param pathGroup        the path group to follow
     * @param eventMap         the event map to use
     * @param useAllianceColor whether to use the alliance color
     * @return the command
     */
    public static SequentialCommandGroup getFollowPathGroupCommand(List<PathPlannerTrajectory> pathGroup, Map<String, Command> eventMap, boolean useAllianceColor) {
        final Command initializeDriveAndPutTargetCommand = new InstantCommand(() -> {
            initializeDrive(false);
            POSE_ESTIMATOR.getField().getObject("target").setPose(pathGroup.get(0).getEndState().poseMeters);
        });
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                POSE_ESTIMATOR::getCurrentPose,
                (pose2d) -> {},
                SWERVE.getKinematics(),
                SWERVE.getTranslationPIDConstants(),
                SWERVE.getRotationPIDConstants(),
                SWERVE::setTargetModuleStates,
                eventMap,
                useAllianceColor,
                SWERVE
        );

        return initializeDriveAndPutTargetCommand.andThen(swerveAutoBuilder.fullAuto(pathGroup));
    }

    /**
     * Creates a command that will drive the robot using the given path and event map.
     *
     * @param path             the path group to follow
     * @param useAllianceColor whether to use the alliance color
     * @return the command
     */
    public static SequentialCommandGroup getFollowPathCommand(PathPlannerTrajectory path, boolean useAllianceColor) {
        final Command initializeDriveAndPutTargetCommand = new InstantCommand(() -> {
            initializeDrive(false);
            POSE_ESTIMATOR.getField().getObject("target").setPose(path.getEndState().poseMeters);
        });
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                POSE_ESTIMATOR::getCurrentPose,
                (pose2d) -> {},
                SWERVE.getKinematics(),
                SWERVE.getTranslationPIDConstants(),
                SWERVE.getRotationPIDConstants(),
                SWERVE::setTargetModuleStates,
                new HashMap<>(),
                useAllianceColor,
                SWERVE
        );

        return initializeDriveAndPutTargetCommand.andThen(swerveAutoBuilder.followPath(path));
    }

    /**
     * Creates a command that sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     * @return the command
     */
    public static InstantCommand getSetSwerveBrakeCommand(boolean brake) {
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
    public static FunctionalCommand getFieldRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                () -> initializeDrive(true),
                () -> SwerveCommands.fieldRelativeDriveFromSuppliers(x, y, theta),
                (interrupted) -> stopDrive(),
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
    public static FunctionalCommand getSelfRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                () -> initializeDrive(true),
                () -> selfRelativeDriveFromSuppliers(x, y, theta),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the field's frame of reference, in open loop mode.
     * All velocities are in percent output from -1 to 1.
     * The angle should be the target angle of the robot, not the target angular velocity.
     * NOT WORKING
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param angle the target angle of the robot
     * @return the command
     */
    @Deprecated
    public static FunctionalCommand getFieldRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> angle) {
        return new FunctionalCommand(
                () -> initializeDrive(false),
                () -> fieldRelativeDriveFromSuppliers(x, y, angle),
                (interrupted) -> stopDrive(),
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
    public static FunctionalCommand getSelfRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                () -> initializeDrive(false),
                () -> selfRelativeDriveFromSuppliers(x, y, theta),
                (interrupted) -> stopDrive(),
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
    public static FunctionalCommand getFieldRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                () -> initializeDrive(false),
                () -> SwerveCommands.fieldRelativeDriveFromSuppliers(x, y, theta),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    private static void initializeDrive(boolean closedLoop) {
        SWERVE.setBrake(true);
        SWERVE.setClosedLoop(closedLoop);
    }

    private static void fieldRelativeDriveFromSuppliers(DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> angle) {
        SWERVE.getRotationController().setGoal(angle.get().getDegrees());
        SWERVE.fieldRelativeDrive(
                new Translation2d(
                        x.getAsDouble() * SWERVE.getMaxSpeedMetersPerSecond(),
                        y.getAsDouble() * SWERVE.getMaxSpeedMetersPerSecond()
                ),
                Rotation2d.fromDegrees(
                        SWERVE.getRotationController().calculate(SWERVE.getHeading().getDegrees())
                )
        );
    }

    private static void fieldRelativeDriveFromSuppliers(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        SWERVE.fieldRelativeDrive(
                new Translation2d(
                        x.getAsDouble() * SWERVE.getMaxSpeedMetersPerSecond(),
                        y.getAsDouble() * SWERVE.getMaxSpeedMetersPerSecond()
                ),
                new Rotation2d(
                        theta.getAsDouble() * SWERVE.getMaxRotationalSpeedRadiansPerSecond()
                )
        );
    }

    private static void selfRelativeDriveFromSuppliers(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        SWERVE.selfRelativeDrive(
                new Translation2d(
                        x.getAsDouble() * SWERVE.getMaxSpeedMetersPerSecond(),
                        y.getAsDouble() * SWERVE.getMaxSpeedMetersPerSecond()
                ),
                new Rotation2d(
                        theta.getAsDouble() * SWERVE.getMaxRotationalSpeedRadiansPerSecond()
                )
        );
    }

    private static void stopDrive() {
        SWERVE.stop();
        SWERVE.setBrake(false);
    }
}

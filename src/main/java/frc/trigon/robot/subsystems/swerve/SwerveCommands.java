package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;

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
        final Pose2d targetPose = getTargetPose(pathGroup.get(pathGroup.size() - 1), useAllianceColor);
        final Command initializeDriveAndPutTargetCommand = new InstantCommand(() -> {
            initializeDrive(false);
            POSE_ESTIMATOR.getField().getObject("target").setPose(targetPose);
        });
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                useAllianceColor ? SwerveCommands::getCurrentAlliancePose : POSE_ESTIMATOR::getCurrentPose,
                useAllianceColor ? SwerveCommands::resetPoseToAlliancePose : POSE_ESTIMATOR::resetPose,
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
     * This cannot use "useAllianceColor" because the current pose that generates this path is not the alliance pose.
     *
     * @param path             the path group to follow
     * @return the command
     */
    public static SequentialCommandGroup getFollowPathCommand(PathPlannerTrajectory path) {
        final Pose2d targetPose = path.getEndState().poseMeters;
        final Command initializeDriveAndPutTargetCommand = new InstantCommand(() -> {
            initializeDrive(false);
            POSE_ESTIMATOR.getField().getObject("target").setPose(targetPose);
        });
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                POSE_ESTIMATOR::getCurrentPose,
                (pose2d) -> {},
                SWERVE.getKinematics(),
                SWERVE.getTranslationPIDConstants(),
                SWERVE.getRotationPIDConstants(),
                SWERVE::setTargetModuleStates,
                new HashMap<>(),
                false,
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
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param angle the target angle of the robot
     * @return the command
     */
    public static FunctionalCommand getFieldRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> angle) {
        return new FunctionalCommand(
                () -> {
                    initializeDrive(false);
                    SWERVE.getRotationController().reset(SWERVE.getHeading().getDegrees());
                },
                () -> fieldRelativeDriveFromSuppliers(x, y, angle),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve to the target pose, and ends when the robot is at the pose with a tolerance.
     *
     * @param targetPose the target pose
     * @return the command
     */
    public static FunctionalCommand getDriveToPoseWithPIDCommand(Pose2d targetPose) {
        final PIDController
                xPIDController = pidConstantsToController(SWERVE.getTranslationPIDConstants()),
                yPIDController = pidConstantsToController(SWERVE.getTranslationPIDConstants()),
                thetaPIDController = pidConstantsToController(SWERVE.getRotationPIDConstants());

        thetaPIDController.enableContinuousInput(-180, 180);

        return new FunctionalCommand(
                () -> {
                    initializeDrive(false);
                    setPosePIDControllersSetpoint(xPIDController, yPIDController, thetaPIDController, targetPose);
                    xPIDController.reset();
                    yPIDController.reset();
                    thetaPIDController.reset();
                },
                () -> driveToFromPosePIDControllers(xPIDController, yPIDController, thetaPIDController),
                (interrupted) -> stopDrive(),
                () -> isAtPose(targetPose),
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

    private static void setPosePIDControllersSetpoint(PIDController xPIDController, PIDController yPIDController, PIDController thetaPIDController, Pose2d targetPose) {
        xPIDController.setSetpoint(targetPose.getTranslation().getX());
        yPIDController.setSetpoint(targetPose.getTranslation().getY());
        thetaPIDController.setSetpoint(targetPose.getRotation().getDegrees());
    }

    private static void driveToFromPosePIDControllers(PIDController xPIDController, PIDController yPIDController, PIDController thetaPIDController) {
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
        final Translation2d driveTranslation = new Translation2d(
                xPIDController.calculate(currentPose.getTranslation().getX()),
                yPIDController.calculate(currentPose.getTranslation().getY())
        );
        final Rotation2d driveRotation = Rotation2d.fromDegrees(
                thetaPIDController.calculate(currentPose.getRotation().getDegrees())
        );
        SWERVE.fieldRelativeDrive(
                driveTranslation,
                driveRotation
        );
    }

    private static boolean isAtPose(Pose2d pose) {
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

        final double
                currentX = currentPose.getTranslation().getX(),
                currentY = currentPose.getTranslation().getY(),
                currentHeading = currentPose.getRotation().getDegrees();

        final double
                targetX = pose.getTranslation().getX(),
                targetY = pose.getTranslation().getY(),
                targetHeading = pose.getRotation().getDegrees();

        return (currentX - targetX <= SWERVE.getTranslationTolerance() &&
                currentY - targetY <= SWERVE.getTranslationTolerance() &&
                currentHeading - targetHeading <= SWERVE.getRotationTolerance()) &&
                (SWERVE.getCurrentVelocity().vxMetersPerSecond < 0.05 &&
                        SWERVE.getCurrentVelocity().vyMetersPerSecond < 0.05 &&
                        SWERVE.getCurrentVelocity().omegaRadiansPerSecond < 0.05);
    }

    private static PIDController pidConstantsToController(PIDConstants pidConstants) {
        return new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, pidConstants.period);
    }

    private static Pose2d getTargetPose(PathPlannerTrajectory path, boolean useAllianceColor) {
        if (!useAllianceColor)
            return path.getEndState().poseMeters;

        final PathPlannerTrajectory transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
        final Pose2d alliancePose = toAlliancePose(transformedTrajectory.getEndState().poseMeters);
        final Transform2d invertDegrees = new Transform2d(new Translation2d(), Rotation2d.fromRotations(0.5));

        return alliancePose.plus(invertDegrees);
    }

    private static void resetPoseToAlliancePose(Pose2d currentPose) {
        POSE_ESTIMATOR.resetPose(toAlliancePose(currentPose));
    }

    private static Pose2d getCurrentAlliancePose() {
        return toAlliancePose(POSE_ESTIMATOR.getCurrentPose());
    }

    private static Pose2d toAlliancePose(Pose2d pose) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
            return pose;

        return new Pose2d(
                FieldConstants.FIELD_LENGTH_METERS - pose.getX(),
                FieldConstants.FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromRotations(0.5))
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

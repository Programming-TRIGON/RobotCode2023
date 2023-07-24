package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.utilities.AllianceUtilities;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    private static final Swerve SWERVE = Swerve.getInstance();
    private static final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();

    /**
     * Creates a command that will turn the robot to the given angle, and end when the robot is at the angle.
     *
     * @param angle the angle to turn to
     * @return the command
     */
    public static FunctionalCommand turnToAngleCommand(Rotation2d angle) {
        final ProfiledPIDController rotationController = SWERVE.getConstants().getRotationController();

        return new FunctionalCommand(
                () -> {
                    initializeDrive(true);
                    rotationController.reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
                },
                () -> fieldRelativeDrive(0, 0, angle),
                (interrupted) -> stopDrive(),
                () -> atAngle(angle)
        );
    }

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
        return getSetSwerveBrakeCommand(true)
                .andThen(new WaitCommand(SWERVE.getConstants().getBrakeTimeSeconds()))
                .andThen(getSetSwerveBrakeCommand(false))
                .ignoringDisable(true);
    }

    /**
     * Creates a command that will drive the robot using the given path group and event map.
     *
     * @param pathGroup the path group to follow
     * @param eventMap  the event map to use
     * @return the command
     */
    public static SequentialCommandGroup getFollowPathGroupCommand(List<PathPlannerTrajectory> pathGroup, Map<String, Command> eventMap) {
        final PathPlannerTrajectory lastPath = pathGroup.get(pathGroup.size() - 1);
        final Command initializeDriveAndShowTargetCommand = new InstantCommand(() -> {
            initializeDrive(false);
            addTargetPoseToField(lastPath, true);
        });
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                POSE_ESTIMATOR::getCurrentPose,
                (pose) -> {},
                SWERVE.getConstants().getKinematics(),
                SWERVE.getConstants().getTranslationPIDConstants(),
                SWERVE.getConstants().getAutoRotationPIDConstants(),
                SWERVE::setTargetModuleStates,
                eventMap,
                true,
                SWERVE
        );

        return initializeDriveAndShowTargetCommand.andThen(swerveAutoBuilder.fullAuto(pathGroup));
    }

    /**
     * Creates a command that will drive the robot using the given path and event map.
     * This cannot use "useAllianceColor" because the current pose that generates this path is not the alliance pose.
     *
     * @param path the path group to follow
     * @return the command
     */
    public static SequentialCommandGroup getFollowPathCommand(PathPlannerTrajectory path) {
        final Command initializeDriveAndShowTargetCommand = new InstantCommand(() -> {
            initializeDrive(true);
            addTargetPoseToField(path, false);
        });
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                POSE_ESTIMATOR::getCurrentPose,
                (pose2d) -> {},
                SWERVE.getConstants().getKinematics(),
                SWERVE.getConstants().getTranslationPIDConstants(),
                SWERVE.getConstants().getRotationPIDConstants(),
                SWERVE::setTargetModuleStates,
                new HashMap<>(),
                false,
                SWERVE
        );

        return initializeDriveAndShowTargetCommand.andThen(swerveAutoBuilder.followPath(path));
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
                () -> fieldRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
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
                () -> selfRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
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
                    SWERVE.getConstants().getRotationController().reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
                },
                () -> fieldRelativeDrive(x.getAsDouble(), y.getAsDouble(), angle.get()),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }
    /**
     * Creates a command that drives the swerve with the given velocities, relative to the field's frame of reference, in closed loop mode.
     * The angle should be the target angle of the robot, not the target angular velocity.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param angle the target angle of the robot
     * @return the command
     */
    public static FunctionalCommand getFieldRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> angle) {
        return new FunctionalCommand(
                () -> {
                    initializeDrive(false);
                    SWERVE.getConstants().getRotationController().reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
                },
                () -> fieldRelativeDrive(x.getAsDouble(), y.getAsDouble(), angle.get()),
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
                xPIDController = pidConstantsToController(SWERVE.getConstants().getTranslationPIDConstants()),
                yPIDController = pidConstantsToController(SWERVE.getConstants().getTranslationPIDConstants()),
                thetaPIDController = pidConstantsToController(SWERVE.getConstants().getRotationPIDConstants());

        thetaPIDController.enableContinuousInput(-180, 180);

        return new FunctionalCommand(
                () -> {
                    initializeDrive(false);
                    initializePosePIDControllers(xPIDController, yPIDController, thetaPIDController, targetPose);
                },
                () -> driveToFromPosePIDControllers(xPIDController, yPIDController, thetaPIDController),
                (interrupted) -> stopDrive(),
                () -> isStoppedAtPose(targetPose),
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
                () -> selfRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
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
                () -> fieldRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    private static boolean atAngle(Rotation2d targetAngle) {
        return Math.abs(POSE_ESTIMATOR.getCurrentPose().getRotation().minus(targetAngle).getDegrees()) < SWERVE.getConstants().getRotationTolerance();
    }

    private static Pose2d getHolonomicPose(PathPlannerTrajectory.PathPlannerState state) {
        return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
    }

    private static void addTargetPoseToField(PathPlannerTrajectory path, boolean useAllianceColor) {
        if (!useAllianceColor) {
            POSE_ESTIMATOR.getField().getObject("target").setPose(getHolonomicPose(path.getEndState()));
            return;
        }

        if (AllianceUtilities.isBlueAlliance()) {
            POSE_ESTIMATOR.getField().getObject("target").setPose(getHolonomicPose(path.getEndState()));
            return;
        }

        final PathPlannerTrajectory transformedPath = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.Alliance.Red);
        POSE_ESTIMATOR.getField().getObject("target").setPose(getHolonomicPose(transformedPath.getEndState()));
    }

    private static void initializePosePIDControllers(PIDController xPIDController, PIDController yPIDController, PIDController thetaPIDController, Pose2d targetPose) {
        setPosePIDControllersSetpoint(xPIDController, yPIDController, thetaPIDController, targetPose);
        xPIDController.reset();
        yPIDController.reset();
        thetaPIDController.reset();
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

    private static boolean isStoppedAtPose(Pose2d pose) {
        return isAtPose(pose) && isSwerveStill();
    }

    private static boolean isAtPose(Pose2d pose) {
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

        final double
                currentX = currentPose.getTranslation().getX(),
                currentY = currentPose.getTranslation().getY(),
                targetX = pose.getTranslation().getX(),
                targetY = pose.getTranslation().getY();
        final Rotation2d
                currentRotation = currentPose.getRotation(),
                targetRotation = pose.getRotation();

        return (currentX - targetX <= SWERVE.getConstants().getTranslationTolerance() &&
                currentY - targetY <= SWERVE.getConstants().getTranslationTolerance() &&
                currentRotation.minus(targetRotation).getDegrees() <= SWERVE.getConstants().getRotationTolerance());
    }

    private static boolean isSwerveStill() {
        return SWERVE.getCurrentVelocity().vxMetersPerSecond < SWERVE.getConstants().getTranslationVelocityTolerance() &&
                SWERVE.getCurrentVelocity().vyMetersPerSecond < SWERVE.getConstants().getTranslationVelocityTolerance() &&
                SWERVE.getCurrentVelocity().omegaRadiansPerSecond < SWERVE.getConstants().getRotationVelocityTolerance();
    }

    private static PIDController pidConstantsToController(PIDConstants pidConstants) {
        return new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, pidConstants.period);
    }

    private static void initializeDrive(boolean closedLoop) {
        SWERVE.setBrake(true);
        SWERVE.setClosedLoop(closedLoop);
    }

    private static void fieldRelativeDrive(double x, double y, Rotation2d angle) {
        SWERVE.getConstants().getRotationController().setGoal(angle.getDegrees());
        SWERVE.fieldRelativeDrive(
                getDriveTranslation(x, y),
                Rotation2d.fromDegrees(
                        SWERVE.getConstants().getRotationController().calculate(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees())
                )
        );
    }

    private static void fieldRelativeDrive(double x, double y, double theta) {
        SWERVE.fieldRelativeDrive(
                getDriveTranslation(x, y),
                getDriveRotation(theta)
        );
    }

    private static void selfRelativeDrive(double x, double y, double theta) {
        SWERVE.selfRelativeDrive(
                getDriveTranslation(x, y),
                getDriveRotation(theta)
        );
    }

    private static Rotation2d getDriveRotation(double rotPower) {
        return new Rotation2d(rotPower * SWERVE.getConstants().getMaxRotationalSpeedRadiansPerSecond());
    }

    private static Translation2d getDriveTranslation(double x, double y) {
        final double xMeterPerSecond = x * SWERVE.getConstants().getMaxSpeedMetersPerSecond();
        final double yMeterPerSecond = y * SWERVE.getConstants().getMaxSpeedMetersPerSecond();

        return new Translation2d(xMeterPerSecond, yMeterPerSecond);
    }

    public static CommandBase getBalanceCommand() {
        return new ProxyCommand(() -> {
            double driveDirection = Math.signum(PoseEstimator.getInstance().getCurrentPose().getX() - 3.777);
            double pitchSignum = Math.signum(-POSE_ESTIMATOR.getCurrentPose().getRotation().getCos());
            final double speed1 = 0.2, speed2 = 0.2;
            return new SequentialCommandGroup(
                    getFieldRelativeClosedLoopSupplierDriveCommand(
                            () -> -speed1 * driveDirection, () -> 0, () -> 0
                    ).until(() -> Math.abs(SWERVE.getPitch()) > 15).deadlineWith(Commands.fakeStaticColor(Color.kFirstBlue)),
                    getFieldRelativeClosedLoopSupplierDriveCommand(
                            () -> -speed2 * driveDirection, () -> 0, () -> 0
                    ).withTimeout(1.5).deadlineWith(Commands.fakeStaticColor(Color.kDeepPink)),
                    getFieldRelativeClosedLoopSupplierDriveCommand(
                            () -> SWERVE.getPitch() * pitchSignum * 0.005, () -> 0, () -> 0
                    ).until(() -> Math.abs(SWERVE.getPitch()) < -1),
                    getLockSwerveCommand().alongWith(Commands.fakeStaticColor(Color.kWhite))
            );
        });

        //        final Command
        //                firstStageLedCommand = Commands.fakeStaticColor(Color.kFirstBlue),
        //                secondStageLedCommand = Commands.fakeStaticColor(Color.kDeepPink),
        //                thirdStageLedCommand = Commands.fakeStaticColor(Color.kWhite),
        //                fourthStageLedCommand = Commands.fakeStaticColor(Color.kDarkGreen);
        //        AtomicBoolean hasClimbed = new AtomicBoolean(false), hasFinished = new AtomicBoolean(false);
        //        AtomicReference<Double> finishTime = new AtomicReference<>((double) 0);
        //        AtomicReference<Double> direction = new AtomicReference<>((double) 0);

        //        return new FunctionalCommand(
        //                () -> {
        //                    hasClimbed.set(false);
        //                    hasFinished.set(false);
        //                    direction.set();
        //                    firstStageLedCommand.schedule();
        //                },
        //                () -> {
        //                    double speed = 0;
        //                    double pitch = SWERVE.getPitch() * Math.signum(-POSE_ESTIMATOR.getCurrentPose().getRotation().getCos()), sign = Math.signum(pitch);
        //                    if(!hasFinished.get()) {
        //                        hasClimbed.set(hasClimbed.get() || pitch * sign > 15);
        //                        if(hasClimbed.get()) {
        //                            speed = pitch * pitch * sign * 0.005;
        //                            secondStageLedCommand.schedule();
        //                        } else {
        //                            speed = direction.get() * -1;
        //                        }
        //
        //                        hasFinished.set(hasClimbed.get() & Math.abs(pitch) < 5);
        //                        finishTime.set(Timer.getFPGATimestamp());
        //                    }
        //                    if(hasFinished.get()) {
        //                        if(Math.abs(pitch) > 3) {
        //                            speed = direction.get() * 1;
        //                            thirdStageLedCommand.schedule();
        //                        } else {
        //                            SWERVE.lockSwerve();
        //                            fourthStageLedCommand.schedule();
        //                        }
        //                    } else
        //                        SWERVE.fieldRelativeDrive(new Translation2d(speed, 0), Rotation2d.fromDegrees(0));
        //                    SmartDashboard.putNumber("balsp", speed);
        //                    SmartDashboard.putBoolean("balhc", hasClimbed.get());
        //                },
        //                (interrupted) -> stopDrive(),
        //                () -> false,
        //                SWERVE
        //        );
    }

    private static void stopDrive() {
        SWERVE.stop();
        SWERVE.setBrake(false);
    }
}

package frc.trigon.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.leds.commands.MovingColorsLedCommand;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.Maths;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Commands {
    private static final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();
    private static final Arm ARM = Arm.getInstance();

    /**
     * Creates a command that follows a path group from an autonomous path's name. This command will also reset the pose.
     *
     * @param pathName the autonomous path's name
     * @return the command
     */
    public static SequentialCommandGroup getAutonomousCommand(String pathName) {
        final List<PathPlannerTrajectory> autonomousPathGroup;
        if (AutonomousConstants.PRELOADED_PATHS.containsKey(pathName)) {
            autonomousPathGroup = AutonomousConstants.PRELOADED_PATHS.get(pathName);
        } else {
            autonomousPathGroup = PathPlanner.loadPathGroup(pathName, AutonomousConstants.AUTONOMOUS_PATH_CONSTRAINS);
            AutonomousConstants.PRELOADED_PATHS.put(pathName, autonomousPathGroup);
        }

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

    /**
     * @return a command that places a cone at the middle note, for autonomous mode
     */
    public static SequentialCommandGroup getPlaceConeAtMidForAutoCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_1).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kRed)),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_2).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                parallel(
                        Gripper.getInstance().getSlowEjectCommand(),
                        SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.2, () -> 0, () -> 0)
                ).withTimeout(0.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * @return a command that places a cone at the high node, for autonomous mode
     */
    public static CommandBase getPlaceCubeAtHighForAutoCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                Gripper.getInstance().getEjectCommand().withTimeout(0.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        ).alongWith(
                SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.15, () -> 0, () -> 0).withTimeout(0.1)
        );
    }

    /**
     * @return a command that places a cube at the high node
     */
    public static CommandBase getPlaceCubeAtHighCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                getWaitForContinueCommand(),
                Gripper.getInstance().getEjectCommand().withTimeout(0.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        ).alongWith(
                new ProxyCommand(SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.15, () -> 0, () -> 0).withTimeout(0.1))
        );
    }

    /**
     * @return a command that places a cube at the high node, for autonomous mode
     */ 
    public static CommandBase getPlaceCubeAtHighCommandForAuto() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                Gripper.getInstance().getEjectCommand().withTimeout(0.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * @return a command that places a cube at the middle node
     */
    public static CommandBase getPlaceCubeAtMidCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_MIDDLE, false, 1.5, 1.5).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                getWaitForContinueCommand(),
                Gripper.getInstance().getEjectCommand().withTimeout(0.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * @return a command that places a cube at the middle node, for autonomous mode
     */
    public static CommandBase getPlaceCubeAtMidForAutoCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_MIDDLE).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                Gripper.getInstance().getEjectCommand().withTimeout(0.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * @return a command that places a cube at the hybrid node
     */
    public static CommandBase getPlaceCubeAtHybridCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HYBRID).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                Gripper.getInstance().getEjectCommand().withTimeout(0.3).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * @return a command that places a cone at the hybrid node
     */
    public static CommandBase getPlaceConeAtHybridCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HYBRID).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                Gripper.getInstance().getEjectCommand().withTimeout(0.3).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * @return a command that places a cone at the high node, for autonomous mode
     */
    public static CommandBase getPlaceConeAtHighForAutoCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.15, () -> 0, () -> 0).withTimeout(0.1).alongWith(
                        ARM.getGoToStateCommand(ArmConstants.ArmStates.AUTO_CONE_HIGH, true, 2, 2)
                                .until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen))),
                SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> 0.1, () -> 0, () -> 0).withTimeout(0.4),
                new WaitCommand(0.2),
                Gripper.getInstance().getEjectCommand().withTimeout(1.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * @return a command that places a cone at the high node
     */
    public static CommandBase getPlaceConeAtHighCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                new ProxyCommand(SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.15, () -> 0, () -> 0)).withTimeout(0.1).alongWith(
                        ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HIGH, true, 2, 2)
                                .until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen))),
                new ProxyCommand(SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> 0.07, () -> 0, () -> 0)).withTimeout(0.23),
                getWaitForContinueCommand(),
                Gripper.getInstance().getEjectCommand().withTimeout(1.5).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    /**
     * Constructs a command that will run the given command, but will not require any subsystems.
     *
     * @param command the command to run
     * @return the command
     */
    public static CommandBase withoutRequirements(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished
        );
    }

    /**
     * Checks if the subsystem is running its default command.
     *
     * @param subsystem the subsystem to check
     * @return whether the default command is running
     */
    public static boolean isRunningDefaultCommand(Subsystem subsystem) {
        return subsystem.getDefaultCommand() == null || subsystem.getDefaultCommand().isScheduled();
    }

    /**
     * Checks if any of the subsystems required by the command are running their default command.
     *
     * @param command the command to check
     * @return whether the default command is running
     */
    public static boolean isRunningDefaultCommand(Command command) {
        if (command == null)
            return true;
        for (Subsystem subsystem : command.getRequirements()) {
            if (isRunningDefaultCommand(subsystem))
                return true;
        }
        return false;
    }

    /**
     * Constructs a command that sets the color of the LEDs to the given color, but as the moving colors command.
     *
     * @param color the color to set the LEDs to
     * @return the command
     */
    public static CommandBase fakeStaticColor(Color color) {
        return new ProxyCommand(new MovingColorsLedCommand(RobotContainer.LEDS, Color.kBlack, 1, 0, color));
    }

    /**
     * @return a command that places a cone at the middle node
     */
    public static SequentialCommandGroup getPlaceConeAtMidCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_1, true, 2, 1.5).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kRed)),
                getWaitForContinueCommand(),
                ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_2, true, 0.5, 1).until(ARM::atGoal).deadlineWith(fakeStaticColor(Color.kDarkGreen)),
                parallel(
                        Gripper.getInstance().getSlowEjectCommand(),
                        new ProxyCommand(SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.2, () -> 0, () -> 0))
                ).withTimeout(0.2).deadlineWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    public static ProxyCommand getDriveAndPlaceCommand(Supplier<Pose2d> alignmentPose, BooleanSupplier isCone, IntSupplier level) {
        return new ProxyCommand(Commands.getDriveToPoseCommand(
                new PathConstraints(1, 1),
                alignmentPose.get()
        ).raceWith(Commands.fakeStaticColor(Color.kYellow)).andThen(
                new ProxyCommand(getPlaceCommand(isCone.getAsBoolean(), level.getAsInt()))
        ));
    }

    public static ProxyCommand getGoToCurrentFirstArmPositionCommand(BooleanSupplier isCone, IntSupplier level) {
        return new ProxyCommand(() -> {
            if (isCone.getAsBoolean())
                return getGoToFirstConePositionCommand(level.getAsInt());

            return getGoToCurrentCubePositionCommand(level.getAsInt());
        });
    }

    public static ProxyCommand getGoToCurrentSecondArmPositionCommand(BooleanSupplier isCone, IntSupplier level) {
        return new ProxyCommand(() -> {
            if (isCone.getAsBoolean())
                return getGoToSecondConePositionCommand(level.getAsInt());

            return getGoToCurrentCubePositionCommand(level.getAsInt());
        });
    }

    private static CommandBase getPlaceCommand(boolean isCone, int level) {
        if(isCone) {
            if(level == 1)
                return Commands.getPlaceConeAtHybridCommand().withName("getPlaceConeAtHybridCommand");
            if(level == 2)
                return Commands.getPlaceConeAtMidCommand().withName("getPlaceConeAtMidCommand");
            if(level == 3)
                return Commands.getPlaceConeAtHighCommand().withName("getPlaceConeAtHighCommand");
        } else {
            if(level == 1)
                return Commands.getPlaceCubeAtHybridCommand().withName("getPlaceCubeAtHybridCommand");
            if(level == 2)
                return Commands.getPlaceCubeAtMidForAutoCommand().withName("getPlaceCubeAtMidCommand");
            if(level == 3)
                return Commands.getPlaceCubeAtHighForAutoCommand().withName("getPlaceCubeAtHighCommand");
        }
        return new InstantCommand();
    }

    private static CommandBase getGoToSecondConePositionCommand(int level) {
        if (level == 1)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HYBRID);
        if (level == 2)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_2);
        if (level == 3)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HIGH);

        return new InstantCommand();
    }

    private static CommandBase getGoToFirstConePositionCommand(int level) {
        if (level == 1)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HYBRID);
        if (level == 2)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_1);
        if (level == 3)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CONE_HIGH);

        return new InstantCommand();
    }

    private static CommandBase getGoToCurrentCubePositionCommand(int level) {
        if (level == 1)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HYBRID);
        if (level == 2)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_MIDDLE);
        if (level == 3)
            return ARM.getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH);

        return new InstantCommand();
    }

    private static CommandBase getDriveToPoseCommand(PathConstraints driveConstraints, Pose2d targetPose) {
        if (!AllianceUtilities.isBlueAlliance()) {
            targetPose = new Pose2d(
                    targetPose.getTranslation().getX(),
                    FieldConstants.FIELD_WIDTH_METERS - targetPose.getTranslation().getY(),
                    targetPose.getRotation().unaryMinus()
            );
        }

        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
        Rotation2d heading = Maths.getAngleBetweenTranslations(
                currentPose.getTranslation(),
                targetPose.getTranslation()
        ).unaryMinus();
        heading = heading.plus(Rotation2d.fromDegrees(45).times(heading.getDegrees() > 0 ? -1 : 1));
        final PathPoint currentPoint = new PathPoint(
                currentPose.getTranslation(),
                heading,
                currentPose.getRotation()
        );
        final PathPoint targetPoint = new PathPoint(
                targetPose.getTranslation(),
                Rotation2d.fromDegrees(180),
                targetPose.getRotation()
        );
        final PathPlannerTrajectory path = PathPlanner.generatePath(driveConstraints, currentPoint, targetPoint);

        return SwerveCommands.getFollowPathCommand(path).andThen(SwerveCommands.getDriveToPoseWithPIDCommand(targetPose));
    }

    private static CommandBase getWaitForContinueCommand() {
        return new WaitUntilCommand(OperatorConstants.CONTINUE_PLACEMENT_TRIGGER);//.deadlineWith(new ProxyCommand(() -> withoutRequirements(Swerve.getInstance().getDefaultCommand())));
    }
}

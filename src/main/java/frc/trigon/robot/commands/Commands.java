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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
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
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

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
        if (!AllianceUtilities.isBlueAlliance()) {
            targetPose = new Pose2d(
                    targetPose.getTranslation().getX(),
                    FieldConstants.FIELD_WIDTH_METERS - targetPose.getTranslation().getY(),
                    targetPose.getRotation().unaryMinus()
            );
        }

        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
        final Rotation2d heading = Maths.getAngleBetweenTranslations(
                currentPose.getTranslation(),
                targetPose.getTranslation()
        ).unaryMinus();
        final PathPoint currentPoint = new PathPoint(
                currentPose.getTranslation(),
                heading,
                currentPose.getRotation()
        );
        final PathPoint targetPoint = new PathPoint(
                targetPose.getTranslation(),
                heading,
                targetPose.getRotation()
        );
        final PathPlannerTrajectory path = PathPlanner.generatePath(driveConstraints, currentPoint, targetPoint);

        return SwerveCommands.getFollowPathCommand(path).andThen(SwerveCommands.getDriveToPoseWithPIDCommand(targetPose));
    }

    public static SequentialCommandGroup getPlaceConeAtMiddleCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                Arm.getInstance().getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_1, true, 1).until(Arm.getInstance()::atGoal).raceWith(fakeStaticColor(Color.kRed)),
                Arm.getInstance().getGoToStateCommand(ArmConstants.ArmStates.CONE_MIDDLE_2, true, 1).until(Arm.getInstance()::atGoal).raceWith(fakeStaticColor(Color.kDarkGreen)),
                parallel(
                        Gripper.getInstance().getSlowEjectCommand(),
                        SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.2, () -> 0, () -> 0)
                ).withTimeout(0.2).raceWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        );
    }

    public static CommandBase getPlaceCubeAtHighCommand() {
        AtomicReference<Double> startTime = new AtomicReference<>((double) 0);
        return new SequentialCommandGroup(
                runOnce(() -> startTime.set(Timer.getFPGATimestamp())),
                Arm.getInstance().getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH_1, true, 1).until(Arm.getInstance()::atGoal).raceWith(fakeStaticColor(Color.kDarkGreen)),
                Gripper.getInstance().getEjectCommand().withTimeout(0.2).raceWith(fakeStaticColor(Color.kDarkBlue)),
                runOnce(() -> SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - startTime.get()))
        ).alongWith(
                SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(() -> -0.15, () -> 0, () -> 0).withTimeout(0.1)
        );
    }

    public static CommandBase fakeStaticColor(Color color) {
        return new MovingColorsLedCommand(RobotContainer.leds, Color.kBlack, 1, 0, color);
    }
}

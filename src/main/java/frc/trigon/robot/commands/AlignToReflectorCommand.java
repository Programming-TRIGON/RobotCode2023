package frc.trigon.robot.commands;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.components.ReflectionLimelight;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class AlignToReflectorCommand extends SequentialCommandGroup {
    private final ReflectionLimelight reflectionLimelight = RobotContainer.REFLECTION_LIMELIGHT;
    final PIDController translationPIDController = pidConstantsToController(new PIDConstants(0.03, 0, 0, 0.02));
    private final ProfiledPIDController rotationController;

    public AlignToReflectorCommand() {
        translationPIDController.setTolerance(0.5, 0.4);
        rotationController = RobotContainer.SWERVE.getRotationController();
        final InstantCommand initializePIDControllerCommand = new InstantCommand(() -> {
            translationPIDController.reset();
            translationPIDController.setSetpoint(0);
            rotationController.reset(PoseEstimator.getInstance().getCurrentPose().getRotation().getDegrees());
            rotationController.setGoal(180);
        });
        final CommandBase turnToDriverStationCommand = SwerveCommands.turnToAngleCommand(Rotation2d.fromDegrees(180));
        final CommandBase alignXInFrontOfReflectorCommand = SwerveCommands.getFieldRelativeClosedLoopSupplierDriveCommand(
                () -> 0,
                () -> MathUtil.clamp(-translationPIDController.calculate(reflectionLimelight.getTx()), -0.2, 0.2),
                () -> 0
        ).until(this::isRobotAlignedToLimelight);
        final CommandBase alignYInFrontOfReflectorCommand = SwerveCommands.getFieldRelativeClosedLoopSupplierDriveCommand(
                () -> -0.2,
                () -> 0,
                () -> 0
        ).until(this::isRobotStopping);

        addCommands(
                initializePIDControllerCommand,
                turnToDriverStationCommand.deadlineWith(Commands.fakeStaticColor(Color.kFirstBlue)),
                alignYInFrontOfReflectorCommand.deadlineWith(Commands.fakeStaticColor(Color.kRed)),
                SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(()->-0.2, ()->0, ()->0).withTimeout(0.2),
                alignXInFrontOfReflectorCommand.deadlineWith(Commands.fakeStaticColor(Color.kDarkGreen)),
                SwerveCommands.getFieldRelativeClosedLoopSupplierDriveCommand(
                        () -> -0.2,
                        () -> 0,
                        () -> 0
                ).until(this::isRobotStopping).deadlineWith(Commands.fakeStaticColor(Color.kRed))
        );
    }

    private double calculateRot() {
        return rotationController.calculate(PoseEstimator.getInstance().getCurrentPose().getRotation().getDegrees());
    }

    private boolean isRobotAlignedToLimelight() {
        return reflectionLimelight.hasTarget() && translationPIDController.atSetpoint();
    }

    private boolean isRobotStopping() {
        return RobotContainer.SWERVE.getGyroYAcceleration() > 4500;
    }

    private PIDController pidConstantsToController(PIDConstants pidConstants) {
        return new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, pidConstants.period);
    }
}

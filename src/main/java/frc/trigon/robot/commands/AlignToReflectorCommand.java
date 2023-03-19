package frc.trigon.robot.commands;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.components.ReflectionLimelight;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class AlignToReflectorCommand extends SequentialCommandGroup {
    private final ReflectionLimelight reflectionLimelight = RobotContainer.REFLECTION_LIMELIGHT;

    public AlignToReflectorCommand() {
        final PIDController translationPIDController = pidConstantsToController(RobotContainer.SWERVE.getTranslationPIDConstants());

        final CommandBase turnToDriverStationCommand = SwerveCommands.turnToAngleCommand(Rotation2d.fromDegrees(0.5));
        final InstantCommand initializePIDControllerCommand = new InstantCommand(() -> {
            translationPIDController.reset();
            translationPIDController.setSetpoint(0);
        });
        final CommandBase alignXInFrontOfReflectorCommand = SwerveCommands.getFieldRelativeClosedLoopSupplierDriveCommand(
                () -> translationPIDController.calculate(reflectionLimelight.getTx()),
                () -> 0,
                () -> 0
        ).until(this::isRobotAlignedToLimelight);
        final CommandBase alignYInFrontOfReflectorCommand = SwerveCommands.getFieldRelativeClosedLoopSupplierDriveCommand(
                () -> 0,
                () -> 1,
                () -> 0
        ).until(this::isRobotStopping);

        addCommands(turnToDriverStationCommand,initializePIDControllerCommand, alignXInFrontOfReflectorCommand, alignYInFrontOfReflectorCommand);
    }

    private boolean isRobotAlignedToLimelight() {
        return reflectionLimelight.hasTarget() && Math.abs(RobotContainer.REFLECTION_LIMELIGHT.getTx()) < 0.5;
    }

    private boolean isRobotStopping() {
        return RobotContainer.SWERVE.getGyroXAcceleration() <= -5;
    }

    private PIDController pidConstantsToController(PIDConstants pidConstants) {
        return new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, pidConstants.period);
    }
}

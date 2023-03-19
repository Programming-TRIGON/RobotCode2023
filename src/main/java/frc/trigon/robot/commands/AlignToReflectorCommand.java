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
    public AlignToReflectorCommand() {
        final ReflectionLimelight reflectionLimelight = RobotContainer.REFLECTION_LIMELIGHT;
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
        ).until(() -> reflectionLimelight.hasTarget() && Math.abs(reflectionLimelight.getTx()) < 0.5);
        final CommandBase alignYInFrontOfReflectorCommand = SwerveCommands.getFieldRelativeClosedLoopSupplierDriveCommand(
                () -> 0,
                () -> 1,
                () -> 0
        ).until(() -> Math.abs(RobotContainer.SWERVE.getGyroXAcceleration()) <= 0.1);

        addCommands(turnToDriverStationCommand,initializePIDControllerCommand, alignXInFrontOfReflectorCommand, alignYInFrontOfReflectorCommand);
    }

    private PIDController pidConstantsToController(PIDConstants pidConstants) {
        return new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, pidConstants.period);
    }
}

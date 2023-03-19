package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.components.ReflectionLimelight;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class AlignToReflectorCommand extends SequentialCommandGroup {
    private static final double IN_FRONT_OF_GRID_Y = 0.5;

    public AlignToReflectorCommand() {
        final ReflectionLimelight reflectionLimelight = RobotContainer.REFLECTION_LIMELIGHT;

        final CommandBase turnToDriverStationCommand = SwerveCommands.turnToAngleCommand(Rotation2d.fromRotations(0.5));
        final CommandBase alignXInFrontOfReflectorCommand = SwerveCommands.driveToTargetSetpointOnYCommand(
                reflectionLimelight::getTx,
                0,
                reflectionLimelight::hasTarget
        );
        final CommandBase alignYInFrontOfReflectorCommand = SwerveCommands.driveToTargetSetpointOnXCommand(
                reflectionLimelight::getTy,
                IN_FRONT_OF_GRID_Y,
                reflectionLimelight::hasTarget
        );

        addCommands(turnToDriverStationCommand, alignXInFrontOfReflectorCommand, alignYInFrontOfReflectorCommand);
    }
}

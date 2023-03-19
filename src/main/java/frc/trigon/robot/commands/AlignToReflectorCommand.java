package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class AlignToReflectorCommand extends SequentialCommandGroup {
    private static final double IN_FRONT_OF_GRID_Y = 0.5;

    public AlignToReflectorCommand() {
        final CommandBase turnToDriverStationCommand = SwerveCommands.turnToAngleCommand(Rotation2d.fromRotations(0.5));
        final CommandBase alignXInFrontOfReflectorCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                () -> 0,
                RobotContainer.REFLECTION_LIMELIGHT::getTx,
                () -> 0
        ).until(this::isXAligned);
        final CommandBase alignYInFrontOfReflectorCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                RobotContainer.REFLECTION_LIMELIGHT::getTy,
                () -> 0,
                () -> 0
        ).until(this::isYAligned);

        addCommands(turnToDriverStationCommand, alignXInFrontOfReflectorCommand, alignYInFrontOfReflectorCommand);
    }

    private boolean isYAligned() {
        return RobotContainer.REFLECTION_LIMELIGHT.hasTarget() && Math.abs(RobotContainer.REFLECTION_LIMELIGHT.getTy() - IN_FRONT_OF_GRID_Y) < 0.5;
    }

    private boolean isXAligned() {
        return RobotContainer.REFLECTION_LIMELIGHT.hasTarget() && Math.abs(RobotContainer.REFLECTION_LIMELIGHT.getTx()) < 0.5;
    }
}

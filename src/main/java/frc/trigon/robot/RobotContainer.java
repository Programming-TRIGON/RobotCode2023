package frc.trigon.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {
    @Log
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    @Log(name = "autoChooser")
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
    private final XboxController driverController = OperatorConstants.DRIVE_CONTROLLER;

    private final CommandBase
            fieldRelativeDriveFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / calculateShiftModeValue(),
                    () -> driverController.getRightX() / calculateShiftModeValue()
            ),
            selfRelativeDriveFromDpadCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> Math.sin(Units.degreesToRadians(-driverController.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> 0
            ),
            resetPoseCommand = new InstantCommand(
                    () -> poseEstimator.resetPose(new Pose2d())
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDrivenAngle
            ),
            fieldRelativeDrivenAngleFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / calculateShiftModeValue(),
                    this::getRightStickAsRotation2d
            );

    public RobotContainer() {
        configureAutonomousChooser();
        setPoseEstimatorPoseSources();
        bindCommands();
    }

    /**
     * @return the command to run in autonomous mode
     */
    CommandBase getAutonomousCommand() {
        if (autonomousPathNameChooser.getSelected() == null)
            return null;

        return Commands.getAutonomousCommand(autonomousPathNameChooser.getSelected());
    }

    private void bindCommands() {
        bindControllerCommands();
        bindDefaultCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_POSE_TRIGGER.onTrue(resetPoseCommand);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_DRIVEN_ANGLE_TRIGGER.onTrue(toggleFieldAndSelfDrivenCommand);
        OperatorConstants.LOCK_SWERVE_TRIGGER.whileTrue(SwerveCommands.getLockSwerveCommand());
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(selfRelativeDriveFromDpadCommand);
    }

    private void configureAutonomousChooser() {
        autonomousPathNameChooser.setDefaultOption("None", null);

        for (String currentPathName : AutonomousConstants.AUTONOMOUS_PATHS_NAMES)
            autonomousPathNameChooser.addOption(currentPathName, currentPathName);
    }

    private double calculateShiftModeValue() {
        final double squaredShiftModeValue = Math.pow(driverController.getRightTriggerAxis(), 2);

        return 1 - squaredShiftModeValue * OperatorConstants.MINIMUM_SHIFT_VALUE_COEFFICIENT;
    }

    @Log(name = "stickDegrees", methodName = "getDegrees")
    private Rotation2d getRightStickAsRotation2d() {
        if (isRightStickStill())
            return SWERVE.getHeading();

        return snapToClosest45Degrees(new Rotation2d(driverController.getRightY(), driverController.getRightX()));
    }

    private Rotation2d snapToClosest45Degrees(Rotation2d rotation2d) {
        return Rotation2d.fromDegrees(Math.round(rotation2d.getDegrees() / 45) * 45);
    }

    private boolean isRightStickStill() {
        return Math.abs(driverController.getRightY()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND&&
                Math.abs(driverController.getRightX()) <= OperatorConstants.DRIVE_CONTROLLER_DEADBAND;
    }

    private void setPoseEstimatorPoseSources() {
        poseEstimator.addRobotPoseSources(CameraConstants.FORWARD_LIMELIGHT);
    }

    private void toggleFieldAndSelfDrivenAngle() {
        if (SWERVE.getDefaultCommand().equals(fieldRelativeDriveFromSticksCommand)) {
            SWERVE.setDefaultCommand(fieldRelativeDrivenAngleFromSticksCommand);
            fieldRelativeDrivenAngleFromSticksCommand.schedule();
        } else {
            SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
            fieldRelativeDriveFromSticksCommand.schedule();
        }
    }
}

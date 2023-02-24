package frc.trigon.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.DriverConstants;
import frc.trigon.robot.robotposesources.AprilTagPhotonCamera;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.testing.TestingSwerve;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {
    @Log
    public static final Swerve SWERVE = TestingSwerve.getInstance();
    @Log
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
//    private final RobotPoseSource testingForwardLimelight = new AprilTagPhotonCamera(
//            "limelight-forward",
//            new Transform3d(
//                    new Translation3d(-0.0355, 0.2, -1.04),
//                    new Rotation3d(0, Math.toRadians(69.1045), Math.toRadians(-7.6346))
//            )
//    );
    private final RobotPoseSource trihardForwardLimelight = new AprilTagPhotonCamera(
            "limelight-forward",
        new Transform3d(
                new Translation3d(0.03, 0, -0.8),
                new Rotation3d(0, Math.toRadians(13), 0)
        )
    );
    private final XboxController driverController = DriverConstants.DRIVE_CONTROLLER;

    private final CommandBase
            fieldRelativeDriveFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    () -> driverController.getLeftY() / calculateShiftModeValue(),
                    () -> driverController.getLeftX() / calculateShiftModeValue(),
                    () -> driverController.getRightX() / calculateShiftModeValue()
            ),
            selfRelativeDriveFromDpadCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(driverController.getPov())) / DriverConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> Math.sin(Units.degreesToRadians(-driverController.getPov())) / DriverConstants.POV_DIVIDER / calculateShiftModeValue(),
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
     * @return the command to run in autonomous mode, from the autonomous chooser
     */
    CommandBase getAutonomousCommand() {
        if (autonomousPathNameChooser.getSelected() == null)
            return null;

        return Commands.getAutonomousCommand(autonomousPathNameChooser.getSelected());
    }

    private double calculateShiftModeValue() {
        final double squaredShiftModeValue = Math.pow(driverController.getRightTriggerAxis(), 2);

        return 1 - squaredShiftModeValue * DriverConstants.MINIMUM_SHIT_VALUE_COEFFICIENT;
    }

    private void configureAutonomousChooser() {
        autonomousPathNameChooser.setDefaultOption("None", null);

        for (String currentPathName : AutonomousConstants.AUTONOMOUS_PATHS_NAMES)
            autonomousPathNameChooser.addOption(currentPathName, currentPathName);
    }

    private void bindCommands() {
        bindControllerCommands();
        bindDefaultCommands();
    }

    private void bindControllerCommands() {
        DriverConstants.RESET_POSE_TRIGGER.onTrue(resetPoseCommand);
        DriverConstants.TOGGLE_FIELD_AND_SELF_DRIVEN_ANGLE_TRIGGER.onTrue(toggleFieldAndSelfDrivenCommand);
        DriverConstants.LOCK_SWERVE_TRIGGER.whileTrue(SwerveCommands.getLockSwerveCommand());
        DriverConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(selfRelativeDriveFromDpadCommand);
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
        return Math.abs(driverController.getRightY()) - DriverConstants.DRIVE_CONTROLLER_DEADBAND <= 0 &&
                Math.abs(driverController.getRightX()) - DriverConstants.DRIVE_CONTROLLER_DEADBAND <= 0;
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
    }

    private void setPoseEstimatorPoseSources() {
        poseEstimator.addRobotPoseSources(trihardForwardLimelight);
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

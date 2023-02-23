package frc.trigon.robot;

import com.pathplanner.lib.PathConstraints;
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
    private final SendableChooser<String> autonomousPathChooser = new SendableChooser<>();
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
    private final RobotPoseSource forwardLimelight = new AprilTagPhotonCamera(
            "limelight-forward",
            new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0)
            )
    );
    private final XboxController driverController = DriverConstants.DRIVE_CONTROLLER;

    private final CommandBase
            fieldRelativeDriveFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    driverController::getLeftY,
                    driverController::getLeftX,
                    driverController::getRightX
            ),
            selfRelativeDriveFromDpadCommand = SwerveCommands.getSelfRelativeOpenLoopSupplierDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(driverController.getPov())) / DriverConstants.POV_DIVIDER,
                    () -> Math.sin(Units.degreesToRadians(-driverController.getPov())) / DriverConstants.POV_DIVIDER,
                    () -> 0
            ),
            resetPoseCommand = new InstantCommand(
                    () -> poseEstimator.resetPose(new Pose2d())
            ),
            toggleFieldAndSelfDrivenCommand = new InstantCommand(
                    this::toggleFieldAndSelfDrivenAngle
            ),
            drive5MetersCommand = Commands.getDriveToPoseCommand(
                    new PathConstraints(1, 0.4),
                    () -> PoseEstimator.getInstance().getCurrentPose().plus(new Transform2d(new Translation2d(5, 0), new Rotation2d())),
                    false
            ),
            fieldRelativeDrivenAngleFromSticksCommand = SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                    driverController::getLeftY,
                    driverController::getLeftX,
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
        if (autonomousPathChooser.getSelected() == null) {
            return null;
        }

        return Commands.getAutonomousCommand(autonomousPathChooser.getSelected());
    }

    private void configureAutonomousChooser() {
        autonomousPathChooser.setDefaultOption("None", null);
        for (String currentPathName : AutonomousConstants.AUTONOMOUS_PATHS_NAMES) {
            autonomousPathChooser.addOption(currentPathName, currentPathName);
        }
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
        DriverConstants.RT_TRIGGER.whileTrue(drive5MetersCommand);
    }

    @Log(name = "stickDegrees", methodName = "getDegrees")
    private Rotation2d getRightStickAsRotation2d() {
        return new Rotation2d(driverController.getRightX(), driverController.getRightY());
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
    }

    private void setPoseEstimatorPoseSources() {
        poseEstimator.addRobotPoseSources(forwardLimelight);
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

package frc.trigon.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.leds.LedStrip;
import frc.trigon.robot.subsystems.leds.MasterLed;
import frc.trigon.robot.subsystems.leds.commands.MovingColorsLEDCommand;
import frc.trigon.robot.subsystems.leds.commands.StaticColorLEDCommand;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;
import io.github.oblarg.oblog.annotations.Log;

import java.util.concurrent.atomic.AtomicInteger;

public class RobotContainer {
    @Log
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    @Log(name = "autoChooser")
    private final SendableChooser<String> autonomousPathNameChooser = new SendableChooser<>();
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
    private final MasterLed masterLed = MasterLed.getInstance();
    private final XboxController driverController = OperatorConstants.DRIVE_CONTROLLER;

    private final LedStrip
            frontLeftLedStrip = new LedStrip(63 * 3, 63, true),
            frontRightLedStrip = new LedStrip(63, 63, true),
            rearLeftLedStrip = new LedStrip(0, 63, false),
            rearRightLedStrip = new LedStrip(63 * 2, 63, false);
//            frontLeftLedStrip = new LedStrip(158, 33, true),
//            frontRightLedStrip = new LedStrip(62, 33, true),
//            rearLeftLedStrip = new LedStrip(0, 62, false),
//            rearRightLedStrip = new LedStrip(95, 63, false);
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
            ),
            redClimbingLEDCommand = new ParallelCommandGroup(
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, frontLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, frontRightLedStrip),
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, rearLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kBlack, Color.kRed, 0.02, 5, rearRightLedStrip)
            ),
            flamesLEDCommand = new ParallelCommandGroup(
                    new MovingColorsLEDCommand(Color.kRed, Color.kYellow, 0.02, 5, frontLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kRed, Color.kYellow, 0.02, 5, frontRightLedStrip),
                    new MovingColorsLEDCommand(Color.kRed, Color.kYellow, 0.02, 5, rearLeftLedStrip),
                    new MovingColorsLEDCommand(Color.kRed, Color.kYellow, 0.02, 5, rearRightLedStrip)
            ),
            purpleAndYellowLEDCommand = new ParallelCommandGroup(
                    new StaticColorLEDCommand(frontLeftLedStrip, new Color[]{Color.kYellow, Color.kPurple}, new int[]{frontLeftLedStrip.getLength() / 2, frontLeftLedStrip.getLength() / 2 + 1}),
                    new StaticColorLEDCommand(frontRightLedStrip, new Color[]{Color.kYellow, Color.kPurple}, new int[]{frontRightLedStrip.getLength() / 2, frontRightLedStrip.getLength() / 2 + 1}),
                    new StaticColorLEDCommand(rearLeftLedStrip, new Color[]{Color.kYellow, Color.kPurple}, new int[]{rearLeftLedStrip.getLength() / 2, rearLeftLedStrip.getLength() / 2 + 1}),
                    new StaticColorLEDCommand(rearRightLedStrip, new Color[]{Color.kYellow, Color.kPurple}, new int[]{rearRightLedStrip.getLength() / 2, rearRightLedStrip.getLength() / 2 + 1})
            );
    AtomicInteger counter = new AtomicInteger(0);

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
        addLedRequirements();
        bindControllerCommands();
        bindDefaultCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(fieldRelativeDriveFromSticksCommand);
        masterLed.setDefaultCommand(flamesLEDCommand);
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

    private void addLedRequirements() {
        flamesLEDCommand.addRequirements(masterLed);
        purpleAndYellowLEDCommand.addRequirements(masterLed);
        redClimbingLEDCommand.addRequirements(masterLed);
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
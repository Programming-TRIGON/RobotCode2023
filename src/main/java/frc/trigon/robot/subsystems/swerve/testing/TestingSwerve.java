package frc.trigon.robot.subsystems.swerve.testing;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.trigon.robot.subsystems.swerve.Swerve;

import java.io.File;

public class TestingSwerve extends Swerve {
    private static final TestingSwerve INSTANCE = new TestingSwerve();

    private TestingSwerve() {
        super();
    }

    public static TestingSwerve getInstance() {
        return INSTANCE;
    }

    @Override
    protected double getDriveNeutralDeadband() {
        return TestingSwerveConstants.DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    protected double getRotationNeutralDeadband() {
        return TestingSwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    public PIDConstants getTranslationPIDConstants() {
        return TestingSwerveConstants.TRANSLATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getAutoRotationPIDConstants() {
        return TestingSwerveConstants.AUTO_ROTATION_PID_CONSTANTS;
    }

    @Override
    protected double getBrakeTimeSeconds() {
        return TestingSwerveConstants.BRAKE_TIME_SECONDS;
    }

    @Override
    public ProfiledPIDController getRotationController() {
        return TestingSwerveConstants.ROTATION_CONTROLLER;
    }

    @Override
    public double getStoppingAcceleration() {
        return TestingSwerveConstants.STOPPING_ACCELERATION;
    }

    @Override
    protected boolean getUseHeadingCorrectionForNormalDrive() {
        return TestingSwerveConstants.USE_HEADING_CORRECTION_FOR_NORMAL_DRIVE;
    }

    @Override
    protected boolean getUseHeadingCorrectionForAuto() {
        return TestingSwerveConstants.USE_HEADING_CORRECTION_FOR_AUTO;
    }

    @Override
    protected File getSwerveDirectory() {
        return TestingSwerveConstants.SWERVE_PATH;
    }

    @Override
    protected SimpleMotorFeedforward getFeedforward() {
        return TestingSwerveConstants.DRIVE_FEEDFORWARD;
    }

    @Override
    protected double getTranslationTolerance() {
        return TestingSwerveConstants.TRANSLATION_TOLERANCE;
    }

    @Override
    protected double getRotationTolerance() {
        return TestingSwerveConstants.ROTATION_TOLERANCE;
    }

    @Override
    protected double getTranslationVelocityTolerance() {
        return TestingSwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    @Override
    protected double getRotationVelocityTolerance() {
        return TestingSwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }
}


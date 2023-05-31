package frc.trigon.robot.subsystems.swerve.trihard;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.utilities.Conversions;
import swervelib.motors.SwerveMotor;

import java.io.File;

public class TrihardSwerve extends Swerve {
    private static final TrihardSwerve INSTANCE = new TrihardSwerve();

    private TrihardSwerve() {
        super();
    }

    public static TrihardSwerve getInstance() {
        return INSTANCE;
    }

    @Override
    protected double getDriveNeutralDeadband() {
        return TrihardSwerveConstants.DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    protected double getRotationNeutralDeadband() {
        return TrihardSwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    protected PIDConstants getAutoRotationPIDConstants() {
        return TrihardSwerveConstants.AUTO_ROTATION_PID_CONSTANTS;
    }

    @Override
    public PIDConstants getTranslationPIDConstants() {
        return TrihardSwerveConstants.TRANSLATION_PID_CONSTANTS;
    }

    @Override
    protected double getBrakeTimeSeconds() {
        return TrihardSwerveConstants.BRAKE_TIME_SECONDS;
    }

    @Override
    public ProfiledPIDController getRotationController() {
        return TrihardSwerveConstants.ROTATION_CONTROLLER;
    }

    @Override
    public double getStoppingAcceleration() {
        return TrihardSwerveConstants.STOPPING_ACCELERATION;
    }

    @Override
    protected boolean getUseHeadingCorrectionForNormalDrive() {
        return TrihardSwerveConstants.USE_HEADING_CORRECTION_FOR_NORMAL_DRIVE;
    }

    @Override
    protected boolean getUseHeadingCorrectionForAuto() {
        return TrihardSwerveConstants.USE_HEADING_CORRECTION_FOR_AUTO;
    }

    @Override
    protected File getSwerveDirectory() {
        return TrihardSwerveConstants.SWERVE_PATH;
    }

    @Override
    protected SimpleMotorFeedforward getFeedforward() {
        return TrihardSwerveConstants.DRIVE_FEEDFORWARD;
    }

    @Override
    protected double getTranslationTolerance() {
        return TrihardSwerveConstants.TRANSLATION_TOLERANCE;
    }

    @Override
    protected double getRotationTolerance() {
        return TrihardSwerveConstants.ROTATION_TOLERANCE;
    }

    @Override
    protected double getTranslationVelocityTolerance() {
        return TrihardSwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    @Override
    protected double getRotationVelocityTolerance() {
        return TrihardSwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }

    private void configAbsoluteAnglePosition() {
        final swervelib.SwerveModule[] swerveModules = getSwerveDrive().getModules();

        // TODO: Check how much you need to multiply by
        swerveModules[0].getAngleMotor().setPosition(SwerveConstants.FRONT_LEFT_ANGLE_ENCODER.getAbsolutePosition() * 360);
        swerveModules[1].getAngleMotor().setPosition(SwerveConstants.FRONT_RIGHT_ANGLE_ENCODER.getAbsolutePosition() * 360);
        swerveModules[2].getAngleMotor().setPosition(SwerveConstants.REAR_LEFT_ANGLE_ENCODER.getAbsolutePosition() * 360);
        swerveModules[3].getAngleMotor().setPosition(SwerveConstants.REAR_RIGHT_ANGLE_ENCODER.getAbsolutePosition() * 360);
    }

    private void setAngleMotorPositionToAbsolute(SwerveMotor angleMotor, DutyCycleEncoder encoder) {
        double motorPosition = Conversions.systemToMotor(encoder.getAbsolutePosition(), getSwerveDrive().);
        angleMotor.setPosition(Conversions.revolutionsToFalconTicks(motorPosition));
    }
}

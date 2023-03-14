package frc.trigon.robot.subsystems.swerve.trihard;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveModule;

public class TrihardSwerve extends Swerve {
    private static final TrihardSwerve INSTANCE = new TrihardSwerve();
    private final TrihardSwerveModule[] swerveModules = TrihardSwerveConstants.SWERVE_MODULES;

    private TrihardSwerve() {
    }

    public static TrihardSwerve getInstance() {
        return INSTANCE;
    }

    @Override
    protected Pigeon2 getGyro() {
        return TrihardSwerveConstants.GYRO;
    }

    @Override
    protected SwerveModule[] getModules() {
        return swerveModules;
    }

    @Override
    protected SwerveDriveKinematics getKinematics() {
        return TrihardSwerveConstants.KINEMATICS;
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
    protected PIDConstants getTranslationPIDConstants() {
        return TrihardSwerveConstants.TRANSLATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getRotationPIDConstants() {
        return TrihardSwerveConstants.ROTATION_PID_CONSTANTS;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return TrihardSwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return TrihardSwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    protected double getBrakeTimeSeconds() {
        return TrihardSwerveConstants.BRAKE_TIME_SECONDS;
    }

    @Override
    protected ProfiledPIDController getRotationController() {
        return TrihardSwerveConstants.ROTATION_CONTROLLER;
    }

    @Override
    protected void lockSwerve() {
        setBrake(true);
        final SwerveModuleState right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        swerveModules[TrihardSwerveModuleConstants.FRONT_LEFT_ID].setTargetState(right);
        swerveModules[TrihardSwerveModuleConstants.FRONT_RIGHT_ID].setTargetState(left);
        swerveModules[TrihardSwerveModuleConstants.REAR_LEFT_ID].setTargetState(left);
        swerveModules[TrihardSwerveModuleConstants.REAR_RIGHT_ID].setTargetState(right);
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

    @Override
    protected SlewRateLimiter getXSlewRateLimiter() {
        return TrihardSwerveConstants.X_SLEW_RATE_LIMITER;
    }

    @Override
    protected SlewRateLimiter getYSlewRateLimiter() {
        return TrihardSwerveConstants.Y_SLEW_RATE_LIMITER;
    }
}

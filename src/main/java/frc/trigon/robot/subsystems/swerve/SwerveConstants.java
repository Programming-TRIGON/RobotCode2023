package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public abstract class SwerveConstants {
    /**
     * @return the swerve's profiled pid controller for rotation
     */
    public abstract ProfiledPIDController getRotationController();

    /**
     * @return the swerve's robot side length in meters
     */
    public abstract double getRobotSideLength();

    /**
     * @return the swerve's module location
     */
    protected abstract Translation2d[] getModuleLocations();

    /**
     * @return the swerve's modules IO
     */
    protected abstract SwerveModuleIO[] getModulesIO();

    /**
     * @return the swerve's kinematics
     */
    public abstract SwerveDriveKinematics getKinematics();

    /**
     * @return the swerve's drive neutral deadband
     */
    protected abstract double getDriveNeutralDeadband();

    /**
     * @return the swerve's rotation neutral deadband
     */
    protected abstract double getRotationNeutralDeadband();

    /**
     * @return the swerve's translation PID constants
     */
    public abstract PIDConstants getTranslationPIDConstants();

    /**
     * @return the swerve's rotation PID constants
     */
    protected abstract PIDConstants getRotationPIDConstants();

    /**
     * @return the swerve's rotation PID constants for auto
     */
    protected abstract PIDConstants getAutoRotationPIDConstants();

    /**
     * @return the swerve's max speed in meters per second
     */
    protected abstract double getMaxSpeedMetersPerSecond();

    /**
     * @return the swerve's max rotational speed in radians per second
     */
    protected abstract double getMaxRotationalSpeedRadiansPerSecond();

    /**
     * @return the swerve's brake time in seconds
     */
    protected abstract double getBrakeTimeSeconds();

    /**
     * @return the tolerance for translation in meters
     */
    protected abstract double getTranslationTolerance();

    /**
     * @return the tolerance for rotation in degrees
     */
    protected abstract double getRotationTolerance();

    /**
     * @return the tolerance for translation velocity in meters per second
     */
    protected abstract double getTranslationVelocityTolerance();

    /**
     * @return the tolerance for rotation velocity in radians per second
     */
    protected abstract double getRotationVelocityTolerance();

    /**
     * @return the slew rate limiter for the x asis
     */
    protected abstract SlewRateLimiter getXSlewRateLimiter();

    /**
     * @return the slew rate limiter for the y asis
     */
    protected abstract SlewRateLimiter getYSlewRateLimiter();
}

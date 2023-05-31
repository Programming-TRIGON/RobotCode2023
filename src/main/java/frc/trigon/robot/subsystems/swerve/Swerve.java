package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.trigon.robot.subsystems.LoggableSubsystemBase;
import io.github.oblarg.oblog.annotations.Log;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

public abstract class Swerve extends LoggableSubsystemBase {
    private SwerveDrive swerveDrive;
    private boolean openLoop = false;

    protected Swerve() {
        configureSwerveDrive();
    }

    /**
     * @return the swerve's profiled pid controller for rotation
     */
    public abstract ProfiledPIDController getRotationController();

    /**
     * @return the acceleration of the swerve when stopping
     */
    public abstract double getStoppingAcceleration();

    /**
     * @return whether the swerve should use heading correction for normal drive
     */
    protected abstract boolean getUseHeadingCorrectionForNormalDrive();

    /**
     * @return whether the swerve should use heading correction for auto
     */
    protected abstract boolean getUseHeadingCorrectionForAuto();

    /**
     * @return the swerve's drive path
     */
    protected abstract File getSwerveDirectory();

    /**
     * @return the swerve's motor feedforward
     */
    protected abstract SimpleMotorFeedforward getFeedforward();

    /**
     * @return the swerve's drive neutral deadband
     */
    protected abstract double getDriveNeutralDeadband();

    /**
     * @return the swerve's rotation neutral deadband
     */
    protected abstract double getRotationNeutralDeadband();

    /**
     * @return the swerve's rotation PID constants for auto
     */
    protected abstract PIDConstants getAutoRotationPIDConstants();

    /**
     * @return the swerve's translation PID constants
     */
    protected abstract PIDConstants getTranslationPIDConstants();

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
     * @return the pitch of the swerve
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * @return the swerve drive
     */
    protected SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Stops the swerve's motors.
     */
    void stop() {
        swerveDrive.drive(new Translation2d(), 0, false, false, false);
    }

    /**
     * @return the swerve's kinematics
     */
    SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Point all modules toward the robot center, thus making the robot very difficult to move.
     * Forcing the robot to keep the current pose.
     */
    void lockPose() {
        swerveDrive.lockPose();
    }

    /**
     * Sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    void setBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Sets whether the swerve drive should be in open loop control, or in closed loop control.
     *
     * @param openLoop true if the drive motor should be in open loop control, false if it should be in closed loop control
     */
    void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }

    /**
     * @return the swerve's rotation PID constants
     */
    PIDConstants getRotationPIDConstants() {
        final double
                p = swerveDrive.getSwerveController().thetaController.getP(),
                i = swerveDrive.getSwerveController().thetaController.getI(),
                d = swerveDrive.getSwerveController().thetaController.getD();

        return new PIDConstants(p, i, d);
    }

    /**
     * @return the heading of the robot
     */
    Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * @return the robot's current velocity
     */
    ChassisSpeeds getCurrentVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Sets the heading of the robot.
     *
     * @param heading the new heading
     */
    void setHeading(Rotation2d heading) {
        swerveDrive.setGyro(new Rotation3d(0, 0, heading.getRadians()));
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference.
     *
     * @param translation          the target x and y velocities in m/s
     * @param rotation             the target theta velocity in radians per second
     * @param useHeadingCorrection whether to correct heading when driving translationally. Set to true to enable
     */
    void selfRelativeDrive(Translation2d translation, Rotation2d rotation, boolean useHeadingCorrection) {
        swerveDrive.drive(translation, rotation.getRadians(), false, openLoop, useHeadingCorrection);
    }

    /**
     * Drives the swerve with the given velocities, relative to the field's frame of reference.
     *
     * @param translation          the target x and y velocities in m/s
     * @param rotation             the target theta velocity in radians per second
     * @param useHeadingCorrection whether to correct heading when driving translationally. Set to true to enable
     */
    void fieldRelativeDrive(Translation2d translation, Rotation2d rotation, boolean useHeadingCorrection) {
        swerveDrive.drive(translation, rotation.getRadians(), true, openLoop, useHeadingCorrection);
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference.
     *
     * @param chassisSpeeds the target velocities for the robot
     */
    void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
     * direction. The other for the angle of the robot.
     *
     * @param x        X joystick input for the robot to move in the X direction.
     * @param y        Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    ChassisSpeeds getTargetSpeeds(double x, double y, double headingX, double headingY) {
        return swerveDrive.getSwerveController().getTargetSpeeds(x, y, headingX, headingY, swerveDrive.getYaw().getRadians());
    }

    /**
     * @return the acceleration of the robot
     */
    public Translation3d getAcceleration() {
        Optional<Translation3d> accel = swerveDrive.getAccel();

        if (accel.isPresent()) {
            return accel.get();
        }

        return new Translation3d();
    }

    /**
     * @return the swerve's max rotational speed in radians per second
     */
    double getMaxRotationalSpeedRadiansPerSecond() {
        return swerveDrive.swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond;
    }

    /**
     * @return the swerve's max speed in meters per second
     */
    double getMaxSpeedMetersPerSecond() {
        return swerveDrive.swerveDriveConfiguration.attainableMaxTranslationalSpeedMetersPerSecond;
    }

    @Log(name = "rotationController/error")
    private double getRotationControllerError() {
        return getRotationController().getPositionError();
    }

    @Log(name = "rotationController/setpoint")
    private double getRotationControllerSetpoint() {
        return getRotationController().getSetpoint().position;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return
                Math.abs(chassisSpeeds.vxMetersPerSecond) <= getDriveNeutralDeadband() &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= getDriveNeutralDeadband() &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= getRotationNeutralDeadband();
    }

    private void configureSwerveDrive() {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(getSwerveDirectory()).createSwerveDrive();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        swerveDrive.replaceSwerveModuleFeedforward(getFeedforward());
    }
}

package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public abstract class Swerve extends SubsystemBase implements Loggable {
    /**
     * @return the swerve's gyro
     */
    protected abstract Pigeon2 getGyro();

    /**
     * @return the swerve's modules
     */
    protected abstract SwerveModule[] getModules();

    /**
     * @return the swerve's kinematics
     */
    protected abstract SwerveDriveKinematics getKinematics();

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
    protected abstract PIDConstants getTranslationPIDConstants();

    /**
     * @return the swerve's rotation PID constants
     */
    protected abstract PIDConstants getRotationPIDConstants();

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
     * @return the swerve's profiled pid controller for rotation
     */
    protected abstract ProfiledPIDController getRotationController();

    /**
     * Locks the swerve, so it'll be hard to move it.
     */
    protected abstract void lockSwerve();

    /**
     * @return the heading of the robot
     */
    @Log(name = "heading", methodName = "getDegrees")
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getGyro().getYaw());
    }

    /**
     * @return the robot's current velocity
     */
    @Log(name = "velocity", methodName = "toString")
    public ChassisSpeeds getCurrentVelocity() {
        final SwerveModuleState[] states = new SwerveModuleState[getModules().length];

        for (int i = 0; i < getModules().length; i++)
            states[i] = getModules()[i].getCurrentState();

        return getKinematics().toChassisSpeeds(states);
    }

    /**
     * Sets the heading of the robot.
     *
     * @param heading the new heading
     */
    public void setHeading(Rotation2d heading) {
        getGyro().setYaw(heading.getDegrees());
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    protected void selfRelativeDrive(Translation2d translation, Rotation2d rotation) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Drives the swerve with the given velocities, relative to the field's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    protected void fieldRelativeDrive(Translation2d translation, Rotation2d rotation) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians(),
                getHeading()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * @return the swerve's module's positions
     */
    protected SwerveModulePosition[] getModulePositions() {
        final SwerveModulePosition[] swerveModuleStates = new SwerveModulePosition[4];
        final SwerveModule[] swerveModules = getModules();

        for (int i = 0; i < swerveModules.length; i++)
            swerveModuleStates[i] = swerveModules[i].getCurrentPosition();

        return swerveModuleStates;
    }

    /**
     * Sets whether the swerve drive should be in closed loop control, or in open loop control.
     *
     * @param closedLoop true if the drive motor should be in closed loop control, false if it should be in open loop control
     */
    protected void setClosedLoop(boolean closedLoop) {
        for (SwerveModule module : getModules())
            module.setDriveMotorClosedLoop(closedLoop);
    }

    /**
     * Stops the swerve's motors.
     */
    protected void stop() {
        for (SwerveModule module : getModules())
            module.stop();
    }

    /**
     * Sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    protected void setBrake(boolean brake) {
        for (SwerveModule module : getModules())
            module.setBrake(brake);
    }

    /**
     * Sets the swerve's target module states.
     *
     * @param swerveModuleStates the target module states
     */
    protected void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < getModules().length; i++)
            getModules()[i].setTargetState(swerveModuleStates[i]);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
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
}

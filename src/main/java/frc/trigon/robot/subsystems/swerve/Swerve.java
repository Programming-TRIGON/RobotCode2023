package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import java.util.ArrayList;
import java.util.List;

public class Swerve extends SubsystemBase {
    private final static Swerve INSTANCE = new Swerve();

    private Swerve() {
        putModulesOnDashboard();
    }

    public static Swerve getInstance() {
        return INSTANCE;
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    void selfRelativeDrive(Translation2d translation, Rotation2d rotation) {
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
    void fieldRelativeDrive(Translation2d translation, Rotation2d rotation) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians(),
                getHeading()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * @return the swerve module positions
     */
    SwerveModulePosition[] getModulePositions() {
        final List<SwerveModulePosition> swerveModuleStates = new ArrayList<>();

        for (SwerveModule currentModule : SwerveConstants.SWERVE_MODULES) {
            swerveModuleStates.add(currentModule.getCurrentModulePosition());
        }

        return swerveModuleStates.toArray(SwerveModulePosition[]::new);
    }

    /**
     * Stops the swerve's motors.
     */
    void stop() {
        for (SwerveModule module : SwerveConstants.SWERVE_MODULES)
            module.stop();
    }

    void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < SwerveConstants.SWERVE_MODULES.length; i++)
            SwerveConstants.SWERVE_MODULES[i].setTargetState(swerveModuleStates[i]);
    }

    /**
     * @return the heading of the robot
     */
    Rotation2d getHeading() {
        return Rotation2d.fromDegrees(SwerveConstants.gyro.getYaw());
    }

    /**
     * Sets the heading of the robot.
     *
     * @param heading the new heading
     */
    @Log
    void setHeading(Rotation2d heading) {
        SwerveConstants.gyro.setYaw(heading.getDegrees());
    }

    /**
     * Sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    void setBrake(boolean brake) {
        for (SwerveModule module : SwerveConstants.SWERVE_MODULES)
            module.setBrake(brake);
    }

    /**
     * @return the robot's current velocity
     */
    ChassisSpeeds getCurrentVelocity() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(
                SwerveConstants.SWERVE_MODULES[0].getCurrentState(),
                SwerveConstants.SWERVE_MODULES[1].getCurrentState(),
                SwerveConstants.SWERVE_MODULES[2].getCurrentState(),
                SwerveConstants.SWERVE_MODULES[3].getCurrentState()
        );
    }

    /**
     * Sets whether the swerve drive should be in closed loop control, or in open loop control.
     *
     * @param closedLoop true if the drive motor should be in closed loop control, false if it should be in open loop control
     */
    void setClosedLoop(boolean closedLoop) {
        for (SwerveModule module : SwerveConstants.SWERVE_MODULES)
            module.setDriveMotorClosedLoop(closedLoop);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }
        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
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
                Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                        Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                        Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private void putModulesOnDashboard() {
        for (int i = 0; i < SwerveConstants.SWERVE_MODULES.length; i++)
            SmartDashboard.putData(
                    getName() + "/" + SwerveModuleConstants.SwerveModules.fromId(i).name(),
                    SwerveConstants.SWERVE_MODULES[i]
            );
    }
}


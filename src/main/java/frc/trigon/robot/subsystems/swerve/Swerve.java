package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

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

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Heading", () -> getHeading().getDegrees(), (heading) -> setHeading(Rotation2d.fromDegrees(heading)));
    }

    private class Commands {
        /**
         * Drives the swerve with the given velocities, relative to the robot's frame of reference.
         * All velocities are in percent output from -1 to 1.
         *
         * @param x     the target forwards velocity
         * @param y     the target leftwards velocity
         * @param theta the target theta velocity, CCW+
         */
        CommandBase cmdSelfRelativeOpenLoopSupplierDrive(
                DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
            return new FunctionalCommand(
                    () -> setBrake(true),
                    () -> selfRelativeDrive(
                            new Translation2d(
                                    x.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                                    y.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                            ),
                            Rotation2d.fromDegrees(
                                    theta.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
                            )
                    ),
                    (interrupted) -> {
                        stop();
                        setBrake(false);
                    },
                    () -> false,
                    Swerve.this
            );
        }

        /**
         * Drives the swerve with the given velocities, relative to the field's frame of reference.
         * All velocities are in percent output from -1 to 1.
         *
         * @param x     the target forwards velocity
         * @param y     the target leftwards velocity
         * @param theta the target theta velocity, CCW+
         */
        CommandBase cmdFieldRelativeOpenLoopSupplierDrive(
                DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
            return new FunctionalCommand(
                    () -> setBrake(true),
                    () -> fieldRelativeDrive(
                            new Translation2d(
                                    x.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                                    y.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                            ),
                            Rotation2d.fromDegrees(
                                    theta.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
                            )
                    ),
                    (interrupted) -> {
                        stop();
                        setBrake(false);
                    },
                    () -> false,
                    Swerve.this
            );
        }
    }
}


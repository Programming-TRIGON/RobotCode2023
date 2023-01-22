package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class SwerveCommands {
    private static final Swerve SWERVE = Swerve.getInstance();

    /**
     * Drives the swerve with the given velocities, relative to the field's frame of reference, in closed loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getFieldRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitialize(true),
                getFieldRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference, in closed loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getSelfRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitialize(true),
                getSelfRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }
    
    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference, in open loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getSelfRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitialize(false),
                getSelfRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Drives the swerve with the given velocities, relative to the field's frame of reference, in open loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static CommandBase getFieldRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                getDriveInitialize(false),
                getFieldRelativeRunnable(x, y, theta),
                getDriveStopConsumer(),
                () -> false,
                SWERVE
        );
    }

    private static Runnable getDriveInitialize(boolean closedLoop) {
        return () -> {
            SWERVE.setBrake(true);
            SWERVE.setClosedLoop(closedLoop);
        };
    }

    private static Runnable getFieldRelativeRunnable(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return () -> SWERVE.fieldRelativeDrive(
                new Translation2d(
                        x.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                        y.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                ),
                Rotation2d.fromDegrees(
                        theta.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
                )
        );
    }

    private static Runnable getSelfRelativeRunnable(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return () -> SWERVE.selfRelativeDrive(
                new Translation2d(
                        x.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                        y.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                ),
                Rotation2d.fromDegrees(
                        theta.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
                )
        );
    }

    private static Consumer<Boolean> getDriveStopConsumer() {
        return (interrupted) -> {
            SWERVE.stop();
            SWERVE.setBrake(false);
        };
    }

}

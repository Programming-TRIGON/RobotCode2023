package frc.trigon.robot.subsystems.swerve.trihard;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
}

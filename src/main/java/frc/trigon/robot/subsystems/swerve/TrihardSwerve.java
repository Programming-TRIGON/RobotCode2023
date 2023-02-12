package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class TrihardSwerve extends Swerve {
    private static final TrihardSwerve INSTANCE = new TrihardSwerve();

    private TrihardSwerve() {
    }

    public static TrihardSwerve getInstance() {
        return INSTANCE;
    }

    @Override
    Pigeon2 getGyro() {
        return TrihardSwerveConstants.GYRO;
    }

    @Override
    SwerveModule[] getModules() {
        return TrihardSwerveConstants.SWERVE_MODULES;
    }

    @Override
    SwerveDriveKinematics getKinematics() {
        return TrihardSwerveConstants.KINEMATICS;
    }

    @Override
    double getDriveNeutralDeadband() {
        return TrihardSwerveConstants.DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    double getRotationNeutralDeadband() {
        return TrihardSwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    PIDConstants getTranslationPIDConstants() {
        return TrihardSwerveConstants.TRANSLATION_PID_CONSTANTS;
    }

    @Override
    PIDConstants getRotationPIDConstants() {
        return TrihardSwerveConstants.ROTATION_PID_CONSTANTS;
    }

    @Override
    double getMaxSpeedMetersPerSecond() {
        return TrihardSwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    double getMaxRotationalSpeedRadiansPerSecond() {
        return TrihardSwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }
}

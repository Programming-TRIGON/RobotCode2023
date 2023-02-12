package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class TestingSwerve extends Swerve {
    private static final TestingSwerve INSTANCE = new TestingSwerve();
    private final TestingSwerveModule[] swerveModules = TestingSwerveConstants.SWERVE_MODULES;

    private TestingSwerve() {
    }

    public static TestingSwerve getInstance() {
        return INSTANCE;
    }

    @Override
    Pigeon2 getGyro() {
        return TestingSwerveConstants.GYRO;
    }

    @Override
    SwerveModule[] getModules() {
        return swerveModules;
    }

    @Override
    SwerveDriveKinematics getKinematics() {
        return TestingSwerveConstants.KINEMATICS;
    }

    @Override
    double getDriveNeutralDeadband() {
        return TestingSwerveConstants.DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    double getRotationNeutralDeadband() {
        return TestingSwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    PIDConstants getTranslationPIDConstants() {
        return TestingSwerveConstants.TRANSLATION_PID_CONSTANTS;
    }

    @Override
    PIDConstants getRotationPIDConstants() {
        return TestingSwerveConstants.ROTATION_PID_CONSTANTS;
    }

    @Override
    double getMaxSpeedMetersPerSecond() {
        return TestingSwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    double getMaxRotationalSpeedRadiansPerSecond() {
        return TestingSwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }
}


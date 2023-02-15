package frc.trigon.robot.subsystems.swerve.testing;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveModule;

public class TestingSwerve extends Swerve {
    private static final TestingSwerve INSTANCE = new TestingSwerve();
    private final TestingSwerveModule[] swerveModules = TestingSwerveConstants.SWERVE_MODULES;

    private TestingSwerve() {
    }

    public static TestingSwerve getInstance() {
        return INSTANCE;
    }

    @Override
    protected Pigeon2 getGyro() {
        return TestingSwerveConstants.GYRO;
    }

    @Override
    protected SwerveModule[] getModules() {
        return swerveModules;
    }

    @Override
    protected SwerveDriveKinematics getKinematics() {
        return TestingSwerveConstants.KINEMATICS;
    }

    @Override
    protected double getDriveNeutralDeadband() {
        return TestingSwerveConstants.DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    protected double getRotationNeutralDeadband() {
        return TestingSwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    protected PIDConstants getTranslationPIDConstants() {
        return TestingSwerveConstants.TRANSLATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getRotationPIDConstants() {
        return TestingSwerveConstants.ROTATION_PID_CONSTANTS;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return TestingSwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return TestingSwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }
}


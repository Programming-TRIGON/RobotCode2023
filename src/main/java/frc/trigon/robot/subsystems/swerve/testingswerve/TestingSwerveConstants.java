package frc.trigon.robot.subsystems.swerve.testingswerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;

public class TestingSwerveConstants extends SwerveConstants {
    private static final double RATE_LIMIT = 5.5;
    static final SlewRateLimiter
            X_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT),
            Y_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT);
    private static final double BRAKE_TIME_SECONDS = 0.3;
    private static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    private static final double
            DRIVE_NEUTRAL_DEADBAND = 0,
            ROTATION_NEUTRAL_DEADBAND = 0;
    static final double
            SIDE_LENGTH_METERS = 0.5,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final Translation2d[] LOCATIONS = {
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(0).location,
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(1).location,
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(2).location,
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(3).location
    };
    static final TestingSwerveModuleIO[] MODULES_IO = {
            new TestingSwerveModuleIO(TestingSwerveModuleConstants.TestingSwerveModules.fromId(0)),
            new TestingSwerveModuleIO(TestingSwerveModuleConstants.TestingSwerveModules.fromId(1)),
            new TestingSwerveModuleIO(TestingSwerveModuleConstants.TestingSwerveModules.fromId(2)),
            new TestingSwerveModuleIO(TestingSwerveModuleConstants.TestingSwerveModules.fromId(3))
    };
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(12, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(15, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(15, 0, 0);
    private static final int PIGEON_ID = 0;
    static final Pigeon2 GYRO = new Pigeon2(PIGEON_ID);
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            720,
            1200
    );
    private static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            10,
            0,
            0,
            ROTATION_CONSTRAINTS
    );
    private static final double
            TRANSLATION_TOLERANCE = 0.2,
            ROTATION_TOLERANCE = 1,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;

    static {
        ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        if (!ConfigurationConstants.IS_REPLAY) {
            GYRO.getConfigurator().apply(new Pigeon2Configuration());

            // TODO: Status signals
//        GYRO.get(PigeonIMU_StatusFrame.CondStatus_1_General, 200);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000);
        }
    }

    @Override
    protected SwerveModuleIO[] getModulesIO() {
        return MODULES_IO;
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return KINEMATICS;
    }

    @Override
    protected double getDriveNeutralDeadband() {
        return DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    protected double getRotationNeutralDeadband() {
        return ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    public PIDConstants getTranslationPIDConstants() {
        return TRANSLATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getRotationPIDConstants() {
        return ROTATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getAutoRotationPIDConstants() {
        return AUTO_ROTATION_PID_CONSTANTS;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    protected double getBrakeTimeSeconds() {
        return BRAKE_TIME_SECONDS;
    }

    @Override
    public ProfiledPIDController getRotationController() {
        return ROTATION_CONTROLLER;
    }

    @Override
    protected double getTranslationTolerance() {
        return TRANSLATION_TOLERANCE;
    }

    @Override
    protected double getRotationTolerance() {
        return ROTATION_TOLERANCE;
    }

    @Override
    protected double getTranslationVelocityTolerance() {
        return TRANSLATION_VELOCITY_TOLERANCE;
    }

    @Override
    protected double getRotationVelocityTolerance() {
        return ROTATION_VELOCITY_TOLERANCE;
    }

    @Override
    public double getRobotSideLength() {
        return SIDE_LENGTH_METERS;
    }

    @Override
    protected Translation2d[] getModuleLocations() {
        return LOCATIONS;
    }

    @Override
    protected SlewRateLimiter getXSlewRateLimiter() {
        return X_SLEW_RATE_LIMITER;
    }

    @Override
    protected SlewRateLimiter getYSlewRateLimiter() {
        return Y_SLEW_RATE_LIMITER;
    }
}

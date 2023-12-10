package frc.trigon.robot.subsystems.swerve.simulationswerve;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;

public class SimulationSwerveConstants extends SwerveConstants {
    private static final double RATE_LIMIT = 5.5;
    static final SlewRateLimiter
            X_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT),
            Y_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT);
    private static final double BRAKE_TIME_SECONDS = 4;
    private static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    private static final double
            DRIVE_NEUTRAL_DEADBAND = 0.1,
            ROTATION_NEUTRAL_DEADBAND = 0;
    static final double
            SIDE_LENGTH_METERS = 0.7,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final Translation2d[] LOCATIONS = {
            SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(0).location,
            SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(1).location,
            SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(2).location,
            SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(3).location
    };
    private static final SimulationSwerveModuleIO[] MODULES_IO = {
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(0)),
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(1)),
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(2)),
            new SimulationSwerveModuleIO(SimulationSwerveModuleConstants.SimulationSwerveModules.fromId(3))
    };
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(12, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(4, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(6, 3, 1);
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            720,
            1200
    );
    private static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            ROTATION_PID_CONSTANTS.kP,
            ROTATION_PID_CONSTANTS.kI,
            ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    private static final double
            TRANSLATION_TOLERANCE = 0.03,
            ROTATION_TOLERANCE = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;

    static {
        ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        ROTATION_CONTROLLER.setIntegratorRange(-30, 30);
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

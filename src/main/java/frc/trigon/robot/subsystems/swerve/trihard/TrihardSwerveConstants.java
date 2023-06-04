package frc.trigon.robot.subsystems.swerve.trihard;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class TrihardSwerveConstants {
    static final double BRAKE_TIME_SECONDS = 4;
    static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.1,
            ROTATION_NEUTRAL_DEADBAND = 0;
    static final double
            SIDE_LENGTH_METERS = 0.7,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final double RATE_LIMIT = 5.5;
    static final SlewRateLimiter
            X_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT),
            Y_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT);
    private static final Translation2d[] LOCATIONS = {
            TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(0).location,
            TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(1).location,
            TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(2).location,
            TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(3).location
    };
    static final TrihardSwerveModule[] SWERVE_MODULES = {
            new TrihardSwerveModule(TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(0)),
            new TrihardSwerveModule(TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(1)),
            new TrihardSwerveModule(TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(2)),
            new TrihardSwerveModule(TrihardSwerveModuleConstants.TrihardSwerveModules.fromId(3))
    };
    static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(3, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(3, 22, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3, 0.0008, 0.5);
    private static final int PIGEON_ID = 0;
    static final Pigeon2 GYRO = new Pigeon2(PIGEON_ID);
    private static final Rotation3d GYRO_MOUNT_POSITION = new Rotation3d(
            Units.degreesToRadians(-0.796127),
            Units.degreesToRadians(-0.95211),
            Units.degreesToRadians(90.0146)
    );
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            720,
            1200
    );
    static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            ROTATION_PID_CONSTANTS.kP,
            ROTATION_PID_CONSTANTS.kI,
            ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    static final double
            TRANSLATION_TOLERANCE = 0.03,
            ROTATION_TOLERANCE = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;

    static {
        ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        ROTATION_CONTROLLER.setIntegratorRange(-30, 30);
        final Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

        gyroConfig.MountPose.MountPoseRoll = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getX());
        gyroConfig.MountPose.MountPosePitch = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getY());
        gyroConfig.MountPose.MountPoseYaw = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getZ());

        GYRO.getConfigurator().apply(gyroConfig);

        // TODO: Status signals
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 200);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 30);

    }
}

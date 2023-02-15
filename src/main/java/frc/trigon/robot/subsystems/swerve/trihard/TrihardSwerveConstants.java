package frc.trigon.robot.subsystems.swerve.trihard;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class TrihardSwerveConstants {
    static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0,
            ROTATION_NEUTRAL_DEADBAND = 0;
    static final double
            SIDE_LENGTH_METERS = 0.7,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
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
            TRANSLATION_PID_CONSTANTS = new PIDConstants(7, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(9, 0, 0);
    private static final int PIGEON_ID = 0;
    static final Pigeon2 GYRO = new Pigeon2(PIGEON_ID);

    static {
        GYRO.configFactoryDefault();

        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 200);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 1000);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 1000);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 1000);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 1000);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 1000);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 1000);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 1000);
        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000);
    }
}

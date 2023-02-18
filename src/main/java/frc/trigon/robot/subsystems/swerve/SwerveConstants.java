package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {
    public static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0,
            ROTATION_NEUTRAL_DEADBAND = 0;
    private static final int PIGEON_ID = 0;
    public static final Pigeon2 gyro = new Pigeon2(PIGEON_ID);
    static final double
            SIDE_LENGTH_METERS = 0.5,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final Translation2d[] LOCATIONS = {
            SwerveModuleConstants.SwerveModules.fromId(0).location,
            SwerveModuleConstants.SwerveModules.fromId(1).location,
            SwerveModuleConstants.SwerveModules.fromId(2).location,
            SwerveModuleConstants.SwerveModules.fromId(3).location
    };
    public static SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(0).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(1).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(2).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(3).swerveModuleConstants)
    };
    static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);

    static {
        gyro.configFactoryDefault();

        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 200);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 1000);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 1000);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 1000);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 1000);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 1000);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 1000);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 1000);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000);
    }
}
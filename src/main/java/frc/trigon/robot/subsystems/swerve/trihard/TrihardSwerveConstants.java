package frc.trigon.robot.subsystems.swerve.trihard;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.trigon.robot.utilities.FilesHandler;

import java.io.File;

public class TrihardSwerveConstants {
    static final double BRAKE_TIME_SECONDS = 4;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.1,
            ROTATION_NEUTRAL_DEADBAND = 0;
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.21564, 2.7054, 0.38437);
    static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(3, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(3, 22, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3, 0.0008, 0.5);
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            720,
            1200
    );
    static final boolean
            USE_HEADING_CORRECTION_FOR_NORMAL_DRIVE = true,
            USE_HEADING_CORRECTION_FOR_AUTO = true;
    private static final double ENCODER_UPDATE_TIME_SECONDS = 3;
    private static final int ENCODER_CHANNEL_OFFSET = 1;
    private static final int
            FRONT_LEFT_ENCODER_CHANNEL = ENCODER_CHANNEL_OFFSET,
            FRONT_RIGHT_ENCODER_CHANNEL = 1 + ENCODER_CHANNEL_OFFSET,
            REAR_LEFT_ENCODER_CHANNEL = 2 + ENCODER_CHANNEL_OFFSET,
            REAR_RIGHT_ENCODER_CHANNEL = 3 + ENCODER_CHANNEL_OFFSET;
    static final DutyCycleEncoder
            FRONT_LEFT_ENCODER = new DutyCycleEncoder(FRONT_LEFT_ENCODER_CHANNEL),
            FRONT_RIGHT_ENCODER = new DutyCycleEncoder(FRONT_RIGHT_ENCODER_CHANNEL),
            REAR_LEFT_ENCODER = new DutyCycleEncoder(REAR_LEFT_ENCODER_CHANNEL),
            REAR_RIGHT_ENCODER = new DutyCycleEncoder(REAR_RIGHT_ENCODER_CHANNEL);
    static final File SWERVE_PATH = new File(FilesHandler.DEPLOY_PATH + "trihardswerve/");
    static final double STOPPING_ACCELERATION = 4500;
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

//        GYRO.configMountPose(90.0146, -0.95211, -0.796127);
    }
}

package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 16;
    static final double
            DRIVE_GEAR_RATIO = 8.14,
            STEER_GEAR_RATIO = 12.8;
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_THEORETICAL_SPEED_METERS_PER_SECOND = 4;

    static final int
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            REAR_LEFT_ID = 2,
            REAR_RIGHT_ID = 3;

    private static final int
            FRONT_LEFT_DRIVE_MOTOR_ID = FRONT_LEFT_ID + 1,
            FRONT_RIGHT_DRIVE_MOTOR_ID = FRONT_RIGHT_ID + 1,
            REAR_LEFT_DRIVE_MOTOR_ID = REAR_LEFT_ID + 1,
            REAR_RIGHT_DRIVE_MOTOR_ID = REAR_RIGHT_ID + 1;
    private static final InvertedValue DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.1,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0.2;
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.21564, 2.7054, 0.38437);
    static final boolean DRIVE_MOTOR_FOC = true;
    private static final TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new TalonFX(FRONT_LEFT_DRIVE_MOTOR_ID),
            FRONT_RIGHT_DRIVE_MOTOR = new TalonFX(FRONT_RIGHT_DRIVE_MOTOR_ID),
            REAR_LEFT_DRIVE_MOTOR = new TalonFX(REAR_LEFT_DRIVE_MOTOR_ID),
            REAR_RIGHT_DRIVE_MOTOR = new TalonFX(REAR_RIGHT_DRIVE_MOTOR_ID);

    private static final int
            FRONT_LEFT_STEER_MOTOR_ID = FRONT_LEFT_ID + 5,
            FRONT_RIGHT_STEER_MOTOR_ID = FRONT_RIGHT_ID + 5,
            REAR_LEFT_STEER_MOTOR_ID = REAR_LEFT_ID + 5,
            REAR_RIGHT_STEER_MOTOR_ID = REAR_RIGHT_ID + 5;
    private static final InvertedValue STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double
            STEER_MOTOR_P = 8.408,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final TalonFX
            FRONT_LEFT_STEER_MOTOR = new TalonFX(FRONT_LEFT_STEER_MOTOR_ID),
            FRONT_RIGHT_STEER_MOTOR = new TalonFX(FRONT_RIGHT_STEER_MOTOR_ID),
            REAR_LEFT_STEER_MOTOR = new TalonFX(REAR_LEFT_STEER_MOTOR_ID),
            REAR_RIGHT_STEER_MOTOR = new TalonFX(REAR_RIGHT_STEER_MOTOR_ID);

    private static final double ENCODER_UPDATE_TIME_SECONDS = 3;
    private static final int ENCODER_CHANNEL_OFFSET = 1;
    private static final int
            FRONT_LEFT_ENCODER_CHANNEL = FRONT_LEFT_ID + ENCODER_CHANNEL_OFFSET,
            FRONT_RIGHT_ENCODER_CHANNEL = FRONT_RIGHT_ID + ENCODER_CHANNEL_OFFSET,
            REAR_LEFT_ENCODER_CHANNEL = REAR_LEFT_ID + ENCODER_CHANNEL_OFFSET,
            REAR_RIGHT_ENCODER_CHANNEL = REAR_RIGHT_ID + ENCODER_CHANNEL_OFFSET;
    private static final double
            FRONT_LEFT_ENCODER_OFFSET = Conversions.degreesToRevolutions(311.064148),
            FRONT_RIGHT_ENCODER_OFFSET = Conversions.degreesToRevolutions(299.171448),
            REAR_LEFT_ENCODER_OFFSET = Conversions.degreesToRevolutions(504.691315),
            REAR_RIGHT_ENCODER_OFFSET = Conversions.degreesToRevolutions(-31.997681);
    private static final DutyCycleEncoder
            FRONT_LEFT_ENCODER = new DutyCycleEncoder(FRONT_LEFT_ENCODER_CHANNEL),
            FRONT_RIGHT_ENCODER = new DutyCycleEncoder(FRONT_RIGHT_ENCODER_CHANNEL),
            REAR_LEFT_ENCODER = new DutyCycleEncoder(REAR_LEFT_ENCODER_CHANNEL),
            REAR_RIGHT_ENCODER = new DutyCycleEncoder(REAR_RIGHT_ENCODER_CHANNEL);

    private static final TrihardSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_STEER_MOTOR,
                    FRONT_LEFT_ENCODER,
                    FRONT_LEFT_ENCODER_OFFSET
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR,
                    FRONT_RIGHT_ENCODER,
                    FRONT_RIGHT_ENCODER_OFFSET
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR,
                    REAR_LEFT_ENCODER,
                    REAR_LEFT_ENCODER_OFFSET
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR,
                    REAR_RIGHT_ENCODER,
                    REAR_RIGHT_ENCODER_OFFSET
            );

    private static final Translation2d
            FRONT_LEFT_MODULE_LOCATION = new Translation2d(
                    TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            FRONT_RIGHT_MODULE_LOCATION = new Translation2d(
                    TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_LEFT_MODULE_LOCATION = new Translation2d(
                    -TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_RIGHT_MODULE_LOCATION = new Translation2d(
                    -TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -TrihardSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            );

    final TalonFX driveMotor, steerMotor;
    final DutyCycleEncoder steerEncoder;
    final double encoderOffset;

    public TrihardSwerveModuleConstants(TalonFX driveMotor, TalonFX steerMotor, DutyCycleEncoder steerEncoder, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
        this.encoderOffset = encoderOffset;

        if (!ConfigurationConstants.IS_REPLAY) {
            configureDriveMotor();
            configureSteerMotor();
        }
    }

    private void configureSteerMotor() {
        final TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();

        // TODO: Status signals
//        steerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255); // Applied output
//        steerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10); // Sensor position and velocity
//        steerMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255); // Battery and temperature
//        steerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255); // Motion magic and profiling
//        steerMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255); // Aux pid feedback
//        steerMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255); // Aux pid information
        steerMotorConfig.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        steerMotorConfig.Slot0.kP = STEER_MOTOR_P;
        steerMotorConfig.Slot0.kI = STEER_MOTOR_I;
        steerMotorConfig.Slot0.kD = STEER_MOTOR_D;

        steerMotor.getConfigurator().apply(steerMotorConfig);
        new Notifier(this::setSteerMotorPositionToAbsolute).startSingle(ENCODER_UPDATE_TIME_SECONDS);
    }

    private void configureDriveMotor() {
        final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

        driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;
        driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;
        driveMotorConfig.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;

        driveMotor.getConfigurator().apply(driveMotorConfig);
    }

    private void setSteerMotorPositionToAbsolute() {
        double encoderPosition = Conversions.offsetRead(steerEncoder.getAbsolutePosition(), encoderOffset);
        double motorPosition = Conversions.systemToMotor(encoderPosition, STEER_GEAR_RATIO);
        steerMotor.setRotorPosition(motorPosition);
    }

    enum TrihardSwerveModules {
        FRONT_LEFT(FRONT_LEFT_ID, FRONT_LEFT_SWERVE_MODULE_CONSTANTS, FRONT_LEFT_MODULE_LOCATION),
        FRONT_RIGHT(FRONT_RIGHT_ID, FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_LOCATION),
        REAR_LEFT(REAR_LEFT_ID, REAR_LEFT_SWERVE_MODULE_CONSTANTS, REAR_LEFT_MODULE_LOCATION),
        REAR_RIGHT(REAR_RIGHT_ID, REAR_RIGHT_SWERVE_MODULE_CONSTANTS, REAR_RIGHT_MODULE_LOCATION);

        final int id;
        final TrihardSwerveModuleConstants swerveModuleConstants;
        final Translation2d location;

        TrihardSwerveModules(int id, TrihardSwerveModuleConstants swerveModuleConstants, Translation2d location) {
            this.id = id;
            this.swerveModuleConstants = swerveModuleConstants;
            this.location = location;
        }

        static TrihardSwerveModules fromId(int id) {
            for (TrihardSwerveModules module : values()) {
                if (module.id == id) {
                    return module;
                }
            }

            throw new IndexOutOfBoundsException("No module with id " + id + " exists");
        }
    }
}

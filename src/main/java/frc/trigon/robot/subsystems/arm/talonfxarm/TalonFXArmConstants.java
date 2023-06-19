package frc.trigon.robot.subsystems.arm.talonfxarm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.utilities.Conversions;

public class TalonFXArmConstants {
    static final boolean USE_FOC = false;

    private static final int
            FIRST_JOINT_FIRST_MOTOR_ID = 9,
            FIRST_JOINT_SECOND_MOTOR_ID = 10,
            SECOND_JOINT_MOTOR_ID = 11;

    static final TalonFX
            FIRST_JOINT_MASTER_MOTOR = new TalonFX(FIRST_JOINT_FIRST_MOTOR_ID),
            SECOND_JOINT_MOTOR = new TalonFX(SECOND_JOINT_MOTOR_ID);
    private static final TalonFX FIRST_JOINT_FOLLOWER_MOTOR = new TalonFX(FIRST_JOINT_SECOND_MOTOR_ID);
    private static final CANcoder
            FIRST_JOINT_ENCODER = new CANcoder(FIRST_JOINT_FIRST_MOTOR_ID);
    private static final WPI_TalonSRX
            SECOND_JOINT_ENCODER = new WPI_TalonSRX(SECOND_JOINT_MOTOR_ID + 1);

    static final double
            FIRST_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD = 40,
            FIRST_JOINT_CURRENT_LIMIT_TIME_THRESHOLD = 0.2,
            SECOND_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD = 30,
            SECOND_JOINT_CURRENT_LIMIT_TIME_THRESHOLD = 0.2;

    private static final InvertedValue
            FIRST_JOINT_MASTER_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FIRST_JOINT_FOLLOWER_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            SECOND_JOINT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final SensorDirectionValue FIRST_JOINT_SENSOR_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    static final NeutralModeValue
            FIRST_JOINT_NEUTRAL_MODE = NeutralModeValue.Brake,
            SECOND_JOINT_NEUTRAL_MODE = NeutralModeValue.Brake;

    private static final boolean SECOND_JOINT_SENSOR_PHASE = false;

    private static final double
            // TODO: Check if these values need conversion
            FIRST_JOINT_KS = 1.0169,
            FIRST_JOINT_KV = 0.53077,
            FIRST_JOINT_KA = 0.2618,
            FIRST_JOINT_KG = 0.71665;

    private static final double
            SECOND_JOINT_KS = 0.39331,
            SECOND_JOINT_KV = 1.936,
            SECOND_JOINT_KA = 0.11478,
            SECOND_JOINT_KG = 0.34408;

    static final ArmFeedforward
            FIRST_JOINT_FEEDFORWARD = new ArmFeedforward(
                    FIRST_JOINT_KS,
                    FIRST_JOINT_KG,
                    FIRST_JOINT_KV,
                    FIRST_JOINT_KA
            ),
            SECOND_JOINT_FEEDFORWARD = new ArmFeedforward(
                    SECOND_JOINT_KS,
                    SECOND_JOINT_KG,
                    SECOND_JOINT_KV,
                    SECOND_JOINT_KA
            );

    private static final double
            FIRST_JOINT_NEUTRAL_DEADBAND = 0.02,
            SECOND_JOINT_NEUTRAL_DEADBAND = 0.01;

    private static final double
            FIRST_JOINT_P = 2.6025415444770283, // 0.26025415444770283
            FIRST_JOINT_I = 0,
            FIRST_JOINT_D = 0;

    private static final double
            SECOND_JOINT_P = 1.5,
            SECOND_JOINT_I = 0.001,
            SECOND_JOINT_D = 0,
            SECOND_JOINT_PEAK_CLOSED_LOOP_OUTPUT = 0.5;

    private static final double
            FIRST_JOINT_ENCODER_OFFSET = 0.3576660166015625,
            SECOND_JOINT_ENCODER_OFFSET = 133.287109;

    static {
        configureEncoders();
        configureFirstJointMasterMotor();
        configureFirstJointFollowerMotor();
        configureSecondJointMotor();
//        FIRST_JOINT_MASTER_MOTOR.configAllowableClosedloopError(0, Conversions.degreesToMagTicks(1));
    }

    private static void configureEncoders() {
        final CANcoderConfiguration firstJointEncoderConfig = new CANcoderConfiguration();

        firstJointEncoderConfig.MagnetSensor.SensorDirection = FIRST_JOINT_SENSOR_DIRECTION_VALUE;
        firstJointEncoderConfig.MagnetSensor.MagnetOffset = FIRST_JOINT_ENCODER_OFFSET;
        FIRST_JOINT_ENCODER.getConfigurator().apply(firstJointEncoderConfig);

        SECOND_JOINT_ENCODER.configFactoryDefault();
        SECOND_JOINT_ENCODER.setInverted(SECOND_JOINT_SENSOR_PHASE);

        SECOND_JOINT_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        SECOND_JOINT_ENCODER.setSelectedSensorPosition(Conversions.offsetRead(SECOND_JOINT_ENCODER.getSensorCollection().getPulseWidthPosition(), Conversions.degreesToMagTicks(SECOND_JOINT_ENCODER_OFFSET)));

        SECOND_JOINT_ENCODER.configClosedloopRamp(1);
        SECOND_JOINT_ENCODER.configOpenloopRamp(1);

    }

    private static void configureSecondJointMotor() {
        final TalonFXConfiguration secondJointMotorConfig = new TalonFXConfiguration();

        secondJointMotorConfig.Slot0.kP = SECOND_JOINT_P;
        secondJointMotorConfig.Slot0.kI = SECOND_JOINT_I;
        secondJointMotorConfig.Slot0.kD = SECOND_JOINT_D;

        secondJointMotorConfig.MotorOutput.Inverted = SECOND_JOINT_MOTOR_INVERTED_VALUE;
        secondJointMotorConfig.MotorOutput.NeutralMode = SECOND_JOINT_NEUTRAL_MODE;
        // TODO: check this (voltage)
        secondJointMotorConfig.MotorOutput.DutyCycleNeutralDeadband = SECOND_JOINT_NEUTRAL_DEADBAND;
        secondJointMotorConfig.MotorOutput.PeakForwardDutyCycle = SECOND_JOINT_PEAK_CLOSED_LOOP_OUTPUT;
        secondJointMotorConfig.MotorOutput.PeakReverseDutyCycle = -SECOND_JOINT_PEAK_CLOSED_LOOP_OUTPUT;

        SECOND_JOINT_MOTOR.getConfigurator().apply(secondJointMotorConfig);

        final double absoluteMotorMagTicks = Conversions.systemToMotor(SECOND_JOINT_ENCODER.getSelectedSensorPosition(), ArmConstants.SECOND_JOINT_GEAR_RATIO);
        final double absoluteMotorDegrees = Conversions.magTicksToDegrees(absoluteMotorMagTicks);
        SECOND_JOINT_MOTOR.setRotorPosition(absoluteMotorDegrees);
    }



    private static void configureFirstJointFollowerMotor() {
        final TalonFXConfiguration firstJointFollowerMotorConfig = new TalonFXConfiguration();

        firstJointFollowerMotorConfig.MotorOutput.Inverted = FIRST_JOINT_FOLLOWER_INVERTED_VALUE;

        FIRST_JOINT_FOLLOWER_MOTOR.getConfigurator().apply(firstJointFollowerMotorConfig);
        FIRST_JOINT_FOLLOWER_MOTOR.setControl(new Follower(FIRST_JOINT_MASTER_MOTOR.getDeviceID(), false));
//        FIRST_JOINT_FOLLOWER_MOTOR.setControl(new StrictFollower(FIRST_JOINT_MASTER_MOTOR.getDeviceID()));
    }

    private static void configureFirstJointMasterMotor() {
        final TalonFXConfiguration firstJointMasterMotorConfig = new TalonFXConfiguration();
        firstJointMasterMotorConfig.Slot0.kP = FIRST_JOINT_P;
        firstJointMasterMotorConfig.Slot0.kI = FIRST_JOINT_I;
        firstJointMasterMotorConfig.Slot0.kD = FIRST_JOINT_D;

        firstJointMasterMotorConfig.Feedback.FeedbackRemoteSensorID = FIRST_JOINT_ENCODER.getDeviceID();
        firstJointMasterMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        firstJointMasterMotorConfig.MotorOutput.Inverted = FIRST_JOINT_MASTER_INVERTED_VALUE;
        firstJointMasterMotorConfig.MotorOutput.NeutralMode = FIRST_JOINT_NEUTRAL_MODE;
        // TODO: check this (voltage)
        firstJointMasterMotorConfig.MotorOutput.DutyCycleNeutralDeadband = FIRST_JOINT_NEUTRAL_DEADBAND;

        FIRST_JOINT_MASTER_MOTOR.getConfigurator().apply(firstJointMasterMotorConfig);
    }
}
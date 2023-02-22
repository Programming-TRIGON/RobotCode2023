package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.LargeFreedomBangBangController;

public class ArmConstants {
    static final double
    FIRST_JOINT_HEIGHT = 101.91482,
    FIRST_JOINT_LENGTH = 85.88525,
    SECOND_JOINT_LENGTH = 55;

    private static final int
            FIRST_JOINT_FIRST_MOTOR_ID = 9,
            FIRST_JOINT_SECOND_MOTOR_ID = 10,
            SECOND_JOINT_MOTOR_ID = 11;

    static final WPI_TalonFX
            FIRST_JOINT_FIRST_MOTOR = new WPI_TalonFX(FIRST_JOINT_FIRST_MOTOR_ID),
            SECOND_JOINT_MOTOR = new WPI_TalonFX(SECOND_JOINT_MOTOR_ID);
    private static final WPI_TalonFX FIRST_JOINT_SECOND_MOTOR = new WPI_TalonFX(FIRST_JOINT_SECOND_MOTOR_ID);
    private static final CANCoder
            FIRST_JOINT_ENCODER = new CANCoder(FIRST_JOINT_FIRST_MOTOR_ID);
    private static final WPI_TalonSRX
            SECOND_JOINT_ENCODER = new WPI_TalonSRX(SECOND_JOINT_MOTOR_ID+1);

    private static final double
            BANG_BANG_TOLERANCE = 0.5,
            BANG_BANG_TRIGGER_THRESHOLD = 5;
    static final LargeFreedomBangBangController SECOND_JOINT_BANG_BANG_CONTROLLER = new LargeFreedomBangBangController(
            BANG_BANG_TOLERANCE,
            BANG_BANG_TRIGGER_THRESHOLD
    );
    static final double BANG_BANG_POWER = 1;

    private static final boolean
            FIRST_JOINT_MOTOR_INVERTED = false,
            FIRST_JOINT_SECOND_MOTOR_INVERTED = false,
            SECOND_JOINT_MOTOR_INVERTED = true;

    static final NeutralMode
            FIRST_JOINT_NEUTRAL_MODE = NeutralMode.Brake,
            SECOND_JOINT_NEUTRAL_MODE = NeutralMode.Brake;

    private static final boolean
            FIRST_JOINT_SENSOR_PHASE = false,
            SECOND_JOINT_SENSOR_PHASE = false;

    private static final double
            FIRST_JOINT_NEUTRAL_DEADBAND = 0.02,
            SECOND_JOINT_NEUTRAL_DEADBAND = 0.01;

    private static final double
            FIRST_JOINT_MAX_SPEED_DEGREES_PER_SECOND = 180,
            SECOND_JOINT_MAX_SPEED_DEGREES_PER_SECOND = 90;

    private static final double
            FIRST_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 180,
            SECOND_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 60;

    static final TrapezoidProfile.Constraints
            FIRST_JOINT_CONSTRAINTS = new TrapezoidProfile.Constraints(
            FIRST_JOINT_MAX_SPEED_DEGREES_PER_SECOND,
            FIRST_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED
    ),
            SECOND_JOINT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    SECOND_JOINT_MAX_SPEED_DEGREES_PER_SECOND,
                    SECOND_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED
            );
    private static final double
            FIRST_JOINT_P = 1.3,
            FIRST_JOINT_I = 0,
            FIRST_JOINT_D = 0.0039723,
            FIRST_JOINT_KS = 1.0169,
            FIRST_JOINT_KV = 0.53077,
            FIRST_JOINT_KA = 0.2618,
            FIRST_JOINT_KG = 0.71665;

    private static final double
            SECOND_JOINT_P = 1,
            SECOND_JOINT_I = 0,
            SECOND_JOINT_D = 0,
            SECOND_JOINT_KS = 0.15575,
            SECOND_JOINT_KV = 1.8696,
            SECOND_JOINT_KA = 0.070188,
            SECOND_JOINT_KG = 0.36887;

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
            FIRST_JOINT_ENCODER_OFFSET = 23.390625,
            SECOND_JOINT_ENCODER_OFFSET = 1916;

    static {
        FIRST_JOINT_FIRST_MOTOR.configFactoryDefault();
        FIRST_JOINT_SECOND_MOTOR.configFactoryDefault();
        SECOND_JOINT_MOTOR.configFactoryDefault();

        FIRST_JOINT_ENCODER.configFactoryDefault();
        SECOND_JOINT_ENCODER.configFactoryDefault();

        FIRST_JOINT_FIRST_MOTOR.setInverted(FIRST_JOINT_MOTOR_INVERTED);
        FIRST_JOINT_SECOND_MOTOR.setInverted(FIRST_JOINT_SECOND_MOTOR_INVERTED);
        SECOND_JOINT_MOTOR.setInverted(SECOND_JOINT_MOTOR_INVERTED);

        FIRST_JOINT_SECOND_MOTOR.follow(FIRST_JOINT_FIRST_MOTOR);

        FIRST_JOINT_FIRST_MOTOR.setSensorPhase(FIRST_JOINT_SENSOR_PHASE);
        SECOND_JOINT_MOTOR.setSensorPhase(SECOND_JOINT_SENSOR_PHASE);

        SECOND_JOINT_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        SECOND_JOINT_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 0);

        double rawSecondPos = SECOND_JOINT_ENCODER.getSelectedSensorPosition(1)-SECOND_JOINT_ENCODER_OFFSET;
        if(rawSecondPos > Conversions.degreesToMagTicks(160))
            rawSecondPos-=Conversions.MAG_TICKS;
        SECOND_JOINT_ENCODER.setSelectedSensorPosition(
                rawSecondPos,
                0,
                0
        );

        SECOND_JOINT_ENCODER.configClosedloopRamp(1);
        SECOND_JOINT_ENCODER.configOpenloopRamp(1);

        FIRST_JOINT_ENCODER.setPositionToAbsolute();
        double rawFirstPos = FIRST_JOINT_ENCODER.getAbsolutePosition() + FIRST_JOINT_ENCODER_OFFSET;
        if(rawFirstPos > 200)
            rawFirstPos-=360;
        FIRST_JOINT_ENCODER.setPosition(rawFirstPos);

        FIRST_JOINT_FIRST_MOTOR.configRemoteFeedbackFilter(FIRST_JOINT_ENCODER, 0);
        SECOND_JOINT_MOTOR.configRemoteFeedbackFilter(SECOND_JOINT_ENCODER, 0);

        FIRST_JOINT_FIRST_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        SECOND_JOINT_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

        FIRST_JOINT_FIRST_MOTOR.config_kP(0, FIRST_JOINT_P);
        FIRST_JOINT_FIRST_MOTOR.config_kI(0, FIRST_JOINT_I);
        FIRST_JOINT_FIRST_MOTOR.config_kD(0, FIRST_JOINT_D);
        FIRST_JOINT_FIRST_MOTOR.configAllowableClosedloopError(0, 12);

        SECOND_JOINT_MOTOR.config_kP(0, SECOND_JOINT_P);
        SECOND_JOINT_MOTOR.config_kI(0, SECOND_JOINT_I);
        SECOND_JOINT_MOTOR.config_kD(0, SECOND_JOINT_D);

        FIRST_JOINT_FIRST_MOTOR.setNeutralMode(FIRST_JOINT_NEUTRAL_MODE);
        SECOND_JOINT_MOTOR.setNeutralMode(SECOND_JOINT_NEUTRAL_MODE);

        FIRST_JOINT_FIRST_MOTOR.configNeutralDeadband(FIRST_JOINT_NEUTRAL_DEADBAND);
        SECOND_JOINT_MOTOR.configNeutralDeadband(SECOND_JOINT_NEUTRAL_DEADBAND);
    }
}

package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.CurrentWatcher;

public class ArmConstants {
    private static final double FIRST_JOINT_CLOSED = -79, SECOND_JOINT_CLOSED = 156;
    static final double MINIMUM_END_EFFECTOR_HEIGHT = 20;

    static final double
            FIRST_JOINT_HEIGHT = 101.91482,
            FIRST_JOINT_LENGTH = 85.88525,
            SECOND_JOINT_LENGTH = 55;
    static final Transform2d
            FIRST_JOINT_TO_SECOND_JOINT = new Transform2d(new Translation2d(FIRST_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0)),
            SECOND_JOINT_TO_END_EFFECTOR = new Transform2d(new Translation2d(SECOND_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0));

    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(
            (FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH) * 2 / 100,
            (FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH) * 2 / 100
    );
    private static final MechanismRoot2d ARM_PIVOT_ROOT = ARM_MECHANISM.getRoot(
            "ArmPivot",
            (FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH) / 100,
            FIRST_JOINT_HEIGHT / 100
    );

    static final MechanismLigament2d
            TARGET_FIRST_JOINT_LIGAMENT = ARM_PIVOT_ROOT.append(
            new MechanismLigament2d("TargetFirstJoint", FIRST_JOINT_LENGTH /100, FIRST_JOINT_CLOSED, 10, new Color8Bit(Color.kGray))
    ),
            TARGET_SECOND_JOINT_LIGAMENT = TARGET_FIRST_JOINT_LIGAMENT.append(
                    new MechanismLigament2d("TargetSecondJoint", SECOND_JOINT_LENGTH / 100, SECOND_JOINT_CLOSED, 8, new Color8Bit(Color.kGray))
            ),
            FIRST_JOINT_LIGAMENT = ARM_PIVOT_ROOT.append(
                    new MechanismLigament2d("FirstJoint", FIRST_JOINT_LENGTH /100, FIRST_JOINT_CLOSED, 10, new Color8Bit(Color.kRed))
            ),
            SECOND_JOINT_LIGAMENT = FIRST_JOINT_LIGAMENT.append(
                    new MechanismLigament2d("SecondJoint", SECOND_JOINT_LENGTH / 100, SECOND_JOINT_CLOSED, 8, new Color8Bit(Color.kBlue))
            );

    static final double RETRACTED_DEGREES = 130;
    private static final int
            FIRST_JOINT_MASTER_MOTOR_ID = 9,
            FIRST_JOINT_FOLLOWER_MOTOR_ID = 10,
            SECOND_JOINT_MOTOR_ID = 11;

    static final WPI_TalonFX
            FIRST_JOINT_MASTER_MOTOR = new WPI_TalonFX(FIRST_JOINT_MASTER_MOTOR_ID),
            FIRST_JOINT_FOLLOWER_MOTOR = new WPI_TalonFX(FIRST_JOINT_FOLLOWER_MOTOR_ID),
            SECOND_JOINT_MOTOR = new WPI_TalonFX(SECOND_JOINT_MOTOR_ID);
    private static final CANCoder
            FIRST_JOINT_ENCODER = new CANCoder(FIRST_JOINT_MASTER_MOTOR_ID);
    private static final WPI_TalonSRX
            SECOND_JOINT_ENCODER = new WPI_TalonSRX(SECOND_JOINT_MOTOR_ID + 1);

    static final double
            DESCEND_PROFILE_COMPLETION_PERCENTAGE = 1,
            RISE_PROFILE_COMPLETION_PERCENTAGE = 0.7;

    static final double
            FIRST_JOINT_TOLERANCE = 6,
            FIRST_JOINT_VELOCITY_TOLERANCE = 900,
            SECOND_JOINT_TOLERANCE = 4,
            SECOND_JOINT_VELOCITY_TOLERANCE = 900;

    private static final double
            FIRST_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD = 40,
            FIRST_JOINT_CURRENT_LIMIT_TIME_THRESHOLD = 0.2,
            SECOND_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD = 30,
            SECOND_JOINT_CURRENT_LIMIT_TIME_THRESHOLD = 0.2;

    static final CurrentWatcher.CurrentWatcherConfig
            FIRST_JOINT_CURRENT_LIMIT_CONFIG = new CurrentWatcher.CurrentWatcherConfig(
            FIRST_JOINT_MASTER_MOTOR::getStatorCurrent,
            FIRST_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD,
            FIRST_JOINT_CURRENT_LIMIT_TIME_THRESHOLD
    ),
            SECOND_JOINT_CURRENT_LIMIT_CONFIG = new CurrentWatcher.CurrentWatcherConfig(
                    SECOND_JOINT_MOTOR::getStatorCurrent,
                    SECOND_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD,
                    SECOND_JOINT_CURRENT_LIMIT_TIME_THRESHOLD
            );

    private static final boolean
            FIRST_JOINT_MOTOR_INVERTED = false,
            FIRST_JOINT_SECOND_MOTOR_INVERTED = false,
            SECOND_JOINT_MOTOR_INVERTED = false;

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
            FIRST_JOINT_MAX_SPEED_DEGREES_PER_SECOND = 1600,
            SECOND_JOINT_MAX_SPEED_DEGREES_PER_SECOND = 720;

    private static final double
            FIRST_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 1600,
            SECOND_JOINT_MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 180;

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
            FIRST_JOINT_D = 0,
            FIRST_JOINT_KS = 1.0169,
            FIRST_JOINT_KV = 0.53077,
            FIRST_JOINT_KA = 0.2618,
            FIRST_JOINT_KG = 0.71665;

    private static final double
            SECOND_JOINT_P = 1.5,
            SECOND_JOINT_I = 0.001,
            SECOND_JOINT_D = 0,
            SECOND_JOINT_KS = 0.39331,
            SECOND_JOINT_KV = 1.936,
            SECOND_JOINT_KA = 0.11478,
            SECOND_JOINT_KG = 0.34408,
            SECOND_JOINT_PEAK_CLOSED_LOOP_OUTPUT = 0.5;

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
            FIRST_JOINT_ENCODER_OFFSET = 219.550781 - 90 - 5,
            SECOND_JOINT_ENCODER_OFFSET = -1250;


    static {
        FIRST_JOINT_MASTER_MOTOR.configFactoryDefault();
        FIRST_JOINT_FOLLOWER_MOTOR.configFactoryDefault();
        SECOND_JOINT_MOTOR.configFactoryDefault();

        FIRST_JOINT_ENCODER.configFactoryDefault();
        SECOND_JOINT_ENCODER.configFactoryDefault();

        FIRST_JOINT_MASTER_MOTOR.setInverted(FIRST_JOINT_MOTOR_INVERTED);
        FIRST_JOINT_FOLLOWER_MOTOR.setInverted(FIRST_JOINT_SECOND_MOTOR_INVERTED);
        SECOND_JOINT_MOTOR.setInverted(SECOND_JOINT_MOTOR_INVERTED);

        FIRST_JOINT_FOLLOWER_MOTOR.follow(FIRST_JOINT_MASTER_MOTOR);

        FIRST_JOINT_MASTER_MOTOR.setSensorPhase(FIRST_JOINT_SENSOR_PHASE);
        SECOND_JOINT_ENCODER.setInverted(SECOND_JOINT_SENSOR_PHASE);


        SECOND_JOINT_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000);
        final SensorCollection sensorCollection = SECOND_JOINT_ENCODER.getSensorCollection();
        sensorCollection.setPulseWidthPosition((int) MathUtil.inputModulus(Conversions.offsetRead(sensorCollection.getPulseWidthPosition(), Conversions.degreesToMagTicks(-42.802734 - 5)), -Conversions.MAG_TICKS / 2, Conversions.MAG_TICKS / 2), 0);

        SECOND_JOINT_ENCODER.configClosedloopRamp(1);
        SECOND_JOINT_ENCODER.configOpenloopRamp(1);

        FIRST_JOINT_ENCODER.setPositionToAbsolute();
        double rawFirstPos = FIRST_JOINT_ENCODER.getAbsolutePosition() + FIRST_JOINT_ENCODER_OFFSET;
        if(rawFirstPos > 200)
            rawFirstPos -= 360;
        FIRST_JOINT_ENCODER.setPosition(rawFirstPos);

        FIRST_JOINT_MASTER_MOTOR.configRemoteFeedbackFilter(FIRST_JOINT_ENCODER, 0);
        SECOND_JOINT_MOTOR.configRemoteFeedbackFilter(SECOND_JOINT_ENCODER, 0);

        FIRST_JOINT_MASTER_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        SECOND_JOINT_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

        FIRST_JOINT_MASTER_MOTOR.config_kP(0, FIRST_JOINT_P);
        FIRST_JOINT_MASTER_MOTOR.config_kI(0, FIRST_JOINT_I);
        FIRST_JOINT_MASTER_MOTOR.config_kD(0, FIRST_JOINT_D);
        FIRST_JOINT_MASTER_MOTOR.configAllowableClosedloopError(0, Conversions.degreesToMagTicks(1));

        SECOND_JOINT_MOTOR.config_kP(0, SECOND_JOINT_P);
        SECOND_JOINT_MOTOR.config_kI(0, SECOND_JOINT_I);
        SECOND_JOINT_MOTOR.config_kD(0, SECOND_JOINT_D);
        SECOND_JOINT_MOTOR.configClosedLoopPeakOutput(0, SECOND_JOINT_PEAK_CLOSED_LOOP_OUTPUT);

        FIRST_JOINT_MASTER_MOTOR.setNeutralMode(FIRST_JOINT_NEUTRAL_MODE);
        SECOND_JOINT_MOTOR.setNeutralMode(SECOND_JOINT_NEUTRAL_MODE);

        FIRST_JOINT_MASTER_MOTOR.configNeutralDeadband(FIRST_JOINT_NEUTRAL_DEADBAND);
        SECOND_JOINT_MOTOR.configNeutralDeadband(SECOND_JOINT_NEUTRAL_DEADBAND);
    }

    public enum ArmStates {
        CLOSED(FIRST_JOINT_CLOSED, SECOND_JOINT_CLOSED),
        CLOSED_COLLECTING(FIRST_JOINT_CLOSED, 86.3),
        CLOSED_COLLECTING_STANDING_CONE(FIRST_JOINT_CLOSED, 99),
        CONE_FEEDER(FIRST_JOINT_CLOSED, 143),
        CUBE_MIDDLE(-51, 117),
        CUBE_HIGH(-5, 35),
        CONE_HYBRID(FIRST_JOINT_CLOSED, SECOND_JOINT_CLOSED),
        CUBE_HYBRID(FIRST_JOINT_CLOSED, SECOND_JOINT_CLOSED),
        CONE_MIDDLE_1(-30, 102),
        CONE_MIDDLE_2(-49, 102),
        CONE_HIGH(22, -19),
        AUTO_CONE_HIGH(24, -19);

        public final double firstMotorPosition, secondMotorPosition;

        ArmStates(double firstMotorPosition, double secondMotorPosition) {
            this.firstMotorPosition = firstMotorPosition;
            this.secondMotorPosition = secondMotorPosition;
        }
    }
}

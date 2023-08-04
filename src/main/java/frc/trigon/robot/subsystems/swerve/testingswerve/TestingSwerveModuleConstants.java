package frc.trigon.robot.subsystems.swerve.testingswerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.utilities.Conversions;

public class TestingSwerveModuleConstants {
    static final double DRIVE_GEAR_RATIO = 10.8577633008;
    static final double WHEEL_DIAMETER_METERS = 0.1;
    static final double MAX_THEORETICAL_SPEED_METERS_PER_SECOND = 4;
    private static final double VOLTAGE_COMP_SATURATION = 12;

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
    // TODO: Why is this inverted?
    private static final InvertedValue DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.2,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0.4;
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.0001, 0.0001, 0.0001);
    static final boolean DRIVE_MOTOR_FOC = false;
    private static final TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new TalonFX(
            FRONT_LEFT_DRIVE_MOTOR_ID
    ),
            FRONT_RIGHT_DRIVE_MOTOR = new TalonFX(
                    FRONT_RIGHT_DRIVE_MOTOR_ID
            ),
            REAR_LEFT_DRIVE_MOTOR = new TalonFX(
                    REAR_LEFT_DRIVE_MOTOR_ID
            ),
            REAR_RIGHT_DRIVE_MOTOR = new TalonFX(
                    REAR_RIGHT_DRIVE_MOTOR_ID
            );

    private static final int
            FRONT_LEFT_STEER_MOTOR_ID = FRONT_LEFT_ID + 1,
            FRONT_RIGHT_STEER_MOTOR_ID = FRONT_RIGHT_ID + 1,
            REAR_LEFT_STEER_MOTOR_ID = REAR_LEFT_ID + 1,
            REAR_RIGHT_STEER_MOTOR_ID = REAR_RIGHT_ID + 1;
    private static final boolean STEER_MOTOR_INVERTED = false;
    private static final double
            STEER_MOTOR_P = 0.01,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final CANSparkMax
            FRONT_LEFT_STEER_MOTOR = new CANSparkMax(
                    FRONT_LEFT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            ),
            FRONT_RIGHT_STEER_MOTOR = new CANSparkMax(
                    FRONT_RIGHT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            ),
            REAR_LEFT_STEER_MOTOR = new CANSparkMax(
                    REAR_LEFT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            ),
            REAR_RIGHT_STEER_MOTOR = new CANSparkMax(
                    REAR_RIGHT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            );
    private static final TestingSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new TestingSwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_STEER_MOTOR
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new TestingSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new TestingSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new TestingSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR
            );

    private static final Translation2d
            FRONT_LEFT_MODULE_LOCATION = new Translation2d(
                    TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            FRONT_RIGHT_MODULE_LOCATION = new Translation2d(
                    TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_LEFT_MODULE_LOCATION = new Translation2d(
                    -TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_RIGHT_MODULE_LOCATION = new Translation2d(
                    -TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -TestingSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            );

    final TalonFX driveMotor;
    final CANSparkMax steerMotor;

    public TestingSwerveModuleConstants(TalonFX driveMotor, CANSparkMax steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        if (!ConfigurationConstants.IS_REPLAY) {
            configureDriveMotor();
            configureSteerMotor();
        }
    }

    private void configureSteerMotor() {
        steerMotor.restoreFactoryDefaults();
        steerMotor.setInverted(STEER_MOTOR_INVERTED);
        steerMotor.enableVoltageCompensation(VOLTAGE_COMP_SATURATION);

        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 255); // Applied output
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10); // Motor movement
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10); // Motor position
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000); // Analog sensor
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 1000); // Alternate encoder
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 100); // Duty cycle position

        steerMotor.setSmartCurrentLimit(10);
        steerMotor.getPIDController().setP(STEER_MOTOR_P);
        steerMotor.getPIDController().setI(STEER_MOTOR_I);
        steerMotor.getPIDController().setD(STEER_MOTOR_D);
        steerMotor.getPIDController().setPositionPIDWrappingEnabled(true);
        steerMotor.getPIDController().setPositionPIDWrappingMinInput(0);
        steerMotor.getPIDController().setPositionPIDWrappingMaxInput(Conversions.DEGREES_PER_REVOLUTIONS);
        steerMotor.getPIDController().setFeedbackDevice(steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle));
        steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(Conversions.DEGREES_PER_REVOLUTIONS);

        steerMotor.burnFlash();
    }

    private void configureDriveMotor() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;
        motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;
        motorConfig.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        driveMotor.getConfigurator().apply(motorConfig);
    }

    enum TestingSwerveModules {
        FRONT_LEFT(FRONT_LEFT_ID, FRONT_LEFT_SWERVE_MODULE_CONSTANTS, FRONT_LEFT_MODULE_LOCATION),
        FRONT_RIGHT(FRONT_RIGHT_ID, FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_LOCATION),
        REAR_LEFT(REAR_LEFT_ID, REAR_LEFT_SWERVE_MODULE_CONSTANTS, REAR_LEFT_MODULE_LOCATION),
        REAR_RIGHT(REAR_RIGHT_ID, REAR_RIGHT_SWERVE_MODULE_CONSTANTS, REAR_RIGHT_MODULE_LOCATION);

        final int id;
        final TestingSwerveModuleConstants swerveModuleConstants;
        final Translation2d location;

        TestingSwerveModules(int id, TestingSwerveModuleConstants swerveModuleConstants, Translation2d location) {
            this.id = id;
            this.swerveModuleConstants = swerveModuleConstants;
            this.location = location;
        }

        static TestingSwerveModules fromId(int id) {
            for (TestingSwerveModules module : values()) {
                if (module.id == id) {
                    return module;
                }
            }

            throw new IndexOutOfBoundsException("No module with id " + id + " exists");
        }
    }
}


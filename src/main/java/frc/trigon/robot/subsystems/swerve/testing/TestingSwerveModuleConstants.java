package frc.trigon.robot.subsystems.swerve.testing;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
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
    private static final boolean DRIVE_MOTOR_INVERTED = false;
    private static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.2,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0.4;
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.0001, 0.0001, 0.0001);
    private static final WPI_TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(
                    FRONT_LEFT_DRIVE_MOTOR_ID
            ),
            FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(
                    FRONT_RIGHT_DRIVE_MOTOR_ID
            ),
            REAR_LEFT_DRIVE_MOTOR = new WPI_TalonFX(
                    REAR_LEFT_DRIVE_MOTOR_ID
            ),
            REAR_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(
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

    final WPI_TalonFX driveMotor;
    final CANSparkMax steerMotor;

    public TestingSwerveModuleConstants(WPI_TalonFX driveMotor, CANSparkMax steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        initialConfig();
    }

    private void initialConfig() {
        driveMotor.configFactoryDefault();
        steerMotor.restoreFactoryDefaults();

        steerMotor.setInverted(STEER_MOTOR_INVERTED);
        driveMotor.setInverted(DRIVE_MOTOR_INVERTED);

        driveMotor.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
        driveMotor.enableVoltageCompensation(true);
        steerMotor.enableVoltageCompensation(VOLTAGE_COMP_SATURATION);

        driveMotor.configOpenloopRamp(DRIVE_OPEN_LOOP_RAMP_RATE);
        driveMotor.configClosedloopRamp(DRIVE_CLOSED_LOOP_RAMP_RATE);

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


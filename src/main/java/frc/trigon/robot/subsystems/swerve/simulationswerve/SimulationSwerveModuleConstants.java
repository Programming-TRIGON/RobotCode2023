package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulationSwerveModuleConstants {
    static final double MAX_MOTOR_VOLTAGE = 12;

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
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.21564, 2.7054, 0.38437);
    private static final double DRIVE_MOMENT_OF_INERTIA = 0.1;
    private static final DCMotorSim
            FRONT_LEFT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            FRONT_RIGHT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_LEFT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_RIGHT_DRIVE_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA);

    static final PIDController STEER_MOTOR_PID_CONTROLLER = new PIDController(0.025, 0, 0);
    private static final double STEER_MOMENT_OF_INERTIA = 0.1;
    private static final DCMotorSim
            FRONT_LEFT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            FRONT_RIGHT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_LEFT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_RIGHT_STEER_MOTOR = new DCMotorSim(DCMotor.getFalcon500(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);

    private static final SimulationSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_STEER_MOTOR
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR
            );

    private static final Translation2d
            FRONT_LEFT_MODULE_LOCATION = new Translation2d(
                    SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            FRONT_RIGHT_MODULE_LOCATION = new Translation2d(
                    SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_LEFT_MODULE_LOCATION = new Translation2d(
                    -SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_RIGHT_MODULE_LOCATION = new Translation2d(
                    -SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -SimulationSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            );

    static {
        STEER_MOTOR_PID_CONTROLLER.enableContinuousInput(0, 360);
    }

    final DCMotorSim driveMotor, steerMotor;

    public SimulationSwerveModuleConstants(DCMotorSim driveMotor, DCMotorSim steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }

    enum SimulationSwerveModules {
        FRONT_LEFT(FRONT_LEFT_ID, FRONT_LEFT_SWERVE_MODULE_CONSTANTS, FRONT_LEFT_MODULE_LOCATION),
        FRONT_RIGHT(FRONT_RIGHT_ID, FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_LOCATION),
        REAR_LEFT(REAR_LEFT_ID, REAR_LEFT_SWERVE_MODULE_CONSTANTS, REAR_LEFT_MODULE_LOCATION),
        REAR_RIGHT(REAR_RIGHT_ID, REAR_RIGHT_SWERVE_MODULE_CONSTANTS, REAR_RIGHT_MODULE_LOCATION);

        final int id;
        final SimulationSwerveModuleConstants swerveModuleConstants;
        final Translation2d location;

        SimulationSwerveModules(int id, SimulationSwerveModuleConstants swerveModuleConstants, Translation2d location) {
            this.id = id;
            this.swerveModuleConstants = swerveModuleConstants;
            this.location = location;
        }

        static SimulationSwerveModuleConstants.SimulationSwerveModules fromId(int id) {
            for (SimulationSwerveModuleConstants.SimulationSwerveModules module : values()) {
                if (module.id == id) {
                    return module;
                }
            }

            throw new IndexOutOfBoundsException("No module with id " + id + " exists");
        }
    }
}

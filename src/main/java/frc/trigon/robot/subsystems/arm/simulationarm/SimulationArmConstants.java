package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.arm.ArmConstants;

public class SimulationArmConstants {
    private static final DCMotor
            FIRST_JOINT_MOTOR = DCMotor.getFalcon500(2),
            SECOND_JOINT_MOTOR = DCMotor.getFalcon500(1);
    private static final double FIRST_JOINT_GEAR_RATIO = 200;
    private static final double
            FIRST_JOINT_MASS = 8,
            SECOND_JOINT_MASS = 8;
    private static final double
            FIRST_JOINT_MINIMUM_ANGLE = Units.degreesToRadians(ArmConstants.FIRST_JOINT_CLOSED - 5),
            FIRST_JOINT_MAXIMUM_ANGLE = Units.degreesToRadians(90),
            SECOND_JOINT_MINIMUM_ANGLE = Units.degreesToRadians(-180),
            SECOND_JOINT_MAXIMUM_ANGLE = Units.degreesToRadians(180);

    static final SingleJointedArmSim
            FIRST_JOINT_SIMULATION = new SingleJointedArmSim(
                    FIRST_JOINT_MOTOR, FIRST_JOINT_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ArmConstants.FIRST_JOINT_LENGTH / 100, FIRST_JOINT_MASS),
                    ArmConstants.FIRST_JOINT_LENGTH / 100, FIRST_JOINT_MINIMUM_ANGLE,
                    FIRST_JOINT_MAXIMUM_ANGLE, true
            ),
            SECOND_JOINT_SIMULATION = new SingleJointedArmSim(
                    SECOND_JOINT_MOTOR, ArmConstants.SECOND_JOINT_GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(ArmConstants.FIRST_JOINT_LENGTH / 100, SECOND_JOINT_MASS),
                    ArmConstants.FIRST_JOINT_LENGTH, SECOND_JOINT_MINIMUM_ANGLE,
                    SECOND_JOINT_MAXIMUM_ANGLE, true
            );

    static final PIDController
            FIRST_JOINT_CONTROLLER = new PIDController(0.1, 0, 0),
            SECOND_JOINT_CONTROLLER = new PIDController(0.1, 0, 0);

    static {
        FIRST_JOINT_CONTROLLER.enableContinuousInput(0, 360);
        SECOND_JOINT_CONTROLLER.enableContinuousInput(0, 360);
    }
}

package frc.trigon.robot.subsystems.gripper.simulationgripper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimulationGripperConstants {
    private static final DCMotor MOTOR = DCMotor.getFalcon500(1);
    private static final double GEAR_RATIO = 1;
    private static final double MOMENT_OF_INERTIA = 0.00032;

    static final FlywheelSim MOTOR_SIMULATION = new FlywheelSim(
            MOTOR,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
}

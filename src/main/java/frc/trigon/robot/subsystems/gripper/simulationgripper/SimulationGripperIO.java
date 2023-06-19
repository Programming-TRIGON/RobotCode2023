package frc.trigon.robot.subsystems.gripper.simulationgripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.trigon.robot.subsystems.gripper.GripperIO;
import frc.trigon.robot.subsystems.gripper.GripperInputsAutoLogged;

public class SimulationGripperIO extends GripperIO {
    private final FlywheelSim motorSimulation = SimulationGripperConstants.MOTOR_SIMULATION;

    @Override
    public void updateInputs(GripperInputsAutoLogged inputs) {
        motorSimulation.update(0.02);

        inputs.velocityRevolutionsPerSecond = Units.radiansToRotations(motorSimulation.getAngularVelocityRadPerSec());
        inputs.statorCurrent = motorSimulation.getCurrentDrawAmps();
    }

    @Override
    public void setTargetPower(double power) {
        motorSimulation.setInputVoltage(MathUtil.clamp(power * 12, -12, 12));
    }
}

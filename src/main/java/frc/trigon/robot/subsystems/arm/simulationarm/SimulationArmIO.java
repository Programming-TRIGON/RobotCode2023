package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;

public class SimulationArmIO extends ArmIO {
    private final SingleJointedArmSim
            firstJointSimulation = SimulationArmConstants.FIRST_JOINT_SIMULATION,
            secondJointSimulation = SimulationArmConstants.SECOND_JOINT_SIMULATION;
    private ArmInputsAutoLogged lastInputs = new ArmInputsAutoLogged();
    private double lastFirstJointVoltage, lastSecondJointVoltage;

    public SimulationArmIO() {
        firstJointSimulation.setState(VecBuilder.fill(Units.degreesToRadians(ArmConstants.FIRST_JOINT_CLOSED), 0.0));
        secondJointSimulation.setState(VecBuilder.fill(Units.degreesToRadians(ArmConstants.SECOND_JOINT_CLOSED), 0.0));
    }
    
    private double firstJointPIDOutput, secondJointPIDOutput;

    @Override
    public void updateInputs(ArmInputsAutoLogged inputs) {
        firstJointSimulation.update(0.02);
        secondJointSimulation.update(0.02);

        inputs.firstJointAppliedVoltage = lastFirstJointVoltage;
        inputs.firstJointPositionDegrees = Units.radiansToDegrees(firstJointSimulation.getAngleRads());
        inputs.firstJointVelocityDegreesPerSecond = Units.radiansToDegrees(firstJointSimulation.getVelocityRadPerSec());
        inputs.firstJointStatorCurrent = firstJointSimulation.getCurrentDrawAmps();
        inputs.firstJointSupplyCurrent = inputs.firstJointStatorCurrent;
        inputs.firstJointClosedLoopOutput = firstJointPIDOutput;

        inputs.secondJointAppliedVoltage = lastSecondJointVoltage;
        inputs.secondJointPositionDegrees = Units.radiansToDegrees(secondJointSimulation.getAngleRads());
        inputs.secondJointVelocityDegreesPerSecond = Units.radiansToDegrees(secondJointSimulation.getVelocityRadPerSec());
        inputs.secondJointStatorCurrent = secondJointSimulation.getCurrentDrawAmps();
        inputs.secondJointSupplyCurrent = inputs.secondJointStatorCurrent;
        inputs.secondJointClosedLoopOutput = secondJointPIDOutput;

        lastInputs = inputs;
    }

    @Override
    public void setTargetFirstJointPosition(double position, double velocity) {
        final double scopedPosition = MathUtil.inputModulus(position, 0, 360);
        firstJointPIDOutput = SimulationArmConstants.FIRST_JOINT_CONTROLLER.calculate(
                lastInputs.firstJointPositionDegrees,
                scopedPosition
        );

        setFirstJointVoltage(firstJointPIDOutput);
    }

    @Override
    public void setTargetSecondJointPosition(double position, double velocity) {
        final double scopedPosition = MathUtil.inputModulus(position, 0, 360);
        secondJointPIDOutput = SimulationArmConstants.SECOND_JOINT_CONTROLLER.calculate(
                lastInputs.secondJointPositionDegrees,
                scopedPosition
        );

        setSecondJointVoltage(secondJointPIDOutput);
    }

    @Override
    public void setupLimits(Runnable firstJointCallback, Runnable secondJointCallback) {
        new Trigger(() -> firstJointSimulation.hasHitLowerLimit() || firstJointSimulation.hasHitUpperLimit()).onTrue(new InstantCommand(firstJointCallback));
        new Trigger(() -> secondJointSimulation.hasHitLowerLimit() || secondJointSimulation.hasHitUpperLimit()).onTrue(new InstantCommand(secondJointCallback));
    }

    @Override
    public void stopFirstJoint() {
        firstJointSimulation.setInputVoltage(0);
    }

    @Override
    public void stopSecondJoint() {
        secondJointSimulation.setInputVoltage(0);
    }

    private void setFirstJointVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        firstJointSimulation.setInputVoltage(voltage);
        lastFirstJointVoltage = voltage;
    }

    private void setSecondJointVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        secondJointSimulation.setInputVoltage(voltage);
        lastSecondJointVoltage = voltage;
    }
}

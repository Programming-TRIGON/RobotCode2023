package frc.trigon.robot.subsystems.arm.talonfxarm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.CurrentWatcher;
import org.littletonrobotics.junction.Logger;

public class TalonFXArmIO extends ArmIO {
    private final TalonFX firstJointMotor = TalonFXArmConstants.FIRST_JOINT_MASTER_MOTOR;
    private final WPI_TalonFX secondJointMotor = TalonFXArmConstants.SECOND_JOINT_MOTOR;
    private ArmInputsAutoLogged lastInputs = new ArmInputsAutoLogged();

    @Override
    public void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.firstJointPositionDegrees = Conversions.revolutionsToDegrees(firstJointMotor.getPosition().getValue());
        inputs.firstJointVelocityDegreesPerSecond = Conversions.revolutionsToDegrees(firstJointMotor.getVelocity().getValue());
        inputs.firstJointStatorCurrent = firstJointMotor.getStatorCurrent().getValue();
        inputs.firstJointSupplyCurrent = firstJointMotor.getSupplyCurrent().getValue();
        inputs.firstJointAppliedVoltage = firstJointMotor.getSupplyVoltage().getValue();
        inputs.firstJointClosedLoopOutput = firstJointMotor.getClosedLoopOutput().getValue();

        Logger.getInstance().recordOutput(
                "Arm/secondJointEncoderDegrees",
                Conversions.magTicksToDegrees(TalonFXArmConstants.SECOND_JOINT_ENCODER.getSelectedSensorPosition())
        );
        inputs.secondJointPositionDegrees = Conversions.magTicksToDegrees(secondJointMotor.getSelectedSensorPosition());
        inputs.secondJointVelocityDegreesPerSecond = getSecondJointVelocityDegreesPerSecond();
        inputs.secondJointStatorCurrent = secondJointMotor.getStatorCurrent();
        inputs.secondJointSupplyCurrent = secondJointMotor.getSupplyCurrent();
        inputs.secondJointAppliedVoltage = secondJointMotor.getMotorOutputVoltage();

        lastInputs = inputs;
    }
    @Override
    public void setTargetFirstJointPosition(double position, double velocity) {
        final double feedforward = TalonFXArmConstants.FIRST_JOINT_FEEDFORWARD.calculate(
                Units.degreesToRadians(position),
                Units.degreesToRadians(velocity)
        );

        Logger.getInstance().recordOutput("Arm/firstff", feedforward);

        final PositionVoltage positionVoltage = new PositionVoltage(
                Conversions.degreesToRevolutions(position),
                TalonFXArmConstants.USE_FOC,
                feedforward,
                0,
                false
        );

        firstJointMotor.setControl(positionVoltage);
    }

    @Override
    public void setTargetSecondJointPosition(double position, double velocity) {
        final double feedforward = TalonFXArmConstants.SECOND_JOINT_FEEDFORWARD.calculate(
                Units.degreesToRadians(position),
                Units.degreesToRadians(velocity)
        );

        Logger.getInstance().recordOutput("Arm/secondff", feedforward);
//        position = Conversions.systemToMotor(position, ArmConstants.SECOND_JOINT_GEAR_RATIO);
//
//        final PositionVoltage positionVoltage = new PositionVoltage(
//                Conversions.degreesToRevolutions(position),
//                TalonFXArmConstants.USE_FOC,
//                feedforward,
//                0,
//                false
//        );

//        secondJointMotor.set(ControlMode.Position, Conversions.degreesToMagTicks(position), DemandType.ArbitraryFeedForward, feedforward);
    }

    @Override
    public void setNeutralMode(boolean brake) {
        final NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        final NeutralMode neutralMode = brake ? NeutralMode.Brake : NeutralMode.Coast;
        final MotorOutputConfigs motorConfig = new MotorOutputConfigs();

        firstJointMotor.getConfigurator().refresh(motorConfig);
        motorConfig.NeutralMode = mode;
        firstJointMotor.getConfigurator().apply(motorConfig);

        secondJointMotor.setNeutralMode(neutralMode);

//        secondJointMotor.getConfigurator().refresh(secondMotorConfig);
//        secondMotorConfig.NeutralMode = mode;
//        secondJointMotor.getConfigurator().apply(secondMotorConfig);
    }

    @Override
    public void setupLimits(Runnable firstJointCallback, Runnable secondJointCallback) {
        new CurrentWatcher(
                () -> lastInputs.firstJointStatorCurrent,
                TalonFXArmConstants.FIRST_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD,
                TalonFXArmConstants.FIRST_JOINT_CURRENT_LIMIT_TIME_THRESHOLD,
                firstJointCallback
        );

        new CurrentWatcher(
                () -> lastInputs.secondJointStatorCurrent,
                TalonFXArmConstants.SECOND_JOINT_CURRENT_LIMIT_CURRENT_THRESHOLD,
                TalonFXArmConstants.SECOND_JOINT_CURRENT_LIMIT_TIME_THRESHOLD,
                secondJointCallback
        );
    }

    @Override
    public void stopFirstJoint() {
        firstJointMotor.stopMotor();
    }

    @Override
    public void stopSecondJoint() {
        secondJointMotor.stopMotor();
    }

    private double getSecondJointVelocityDegreesPerSecond() {
        final double velocityMagTicksPer100MS = secondJointMotor.getSelectedSensorVelocity();
        final double velocityDegreesPer100MS = Conversions.magTicksToDegrees(velocityMagTicksPer100MS);
        return Conversions.perHundredMsToPerSecond(velocityDegreesPer100MS);
    }
}

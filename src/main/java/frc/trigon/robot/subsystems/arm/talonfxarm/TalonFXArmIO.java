package frc.trigon.robot.subsystems.arm.talonfxarm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.CurrentWatcher;

public class TalonFXArmIO extends ArmIO {
    private final TalonFX
            firstJointMotor = TalonFXArmConstants.FIRST_JOINT_MASTER_MOTOR,
            secondJointMotor = TalonFXArmConstants.SECOND_JOINT_MOTOR;
    private ArmInputsAutoLogged lastInputs = new ArmInputsAutoLogged();

    @Override
    public void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.firstJointPositionDegrees = Conversions.revolutionsToDegrees(firstJointMotor.getPosition().getValue());
        inputs.firstJointVelocityDegreesPerSecond = Conversions.revolutionsToDegrees(firstJointMotor.getVelocity().getValue());
        inputs.firstJointStatorCurrent = firstJointMotor.getStatorCurrent().getValue();
        inputs.firstJointSupplyCurrent = firstJointMotor.getSupplyCurrent().getValue();

        inputs.secondJointPositionDegrees = secondJointValueToSystemDegrees(secondJointMotor.getPosition().getValue());
        inputs.secondJointVelocityDegreesPerSecond = secondJointValueToSystemDegrees(secondJointMotor.getVelocity().getValue());
        inputs.secondJointStatorCurrent = secondJointMotor.getStatorCurrent().getValue();
        inputs.secondJointSupplyCurrent = secondJointMotor.getSupplyCurrent().getValue();

        lastInputs = inputs;
    }

    @Override
    public void setTargetFirstJointPosition(double position, double velocity) {
        final double feedforward = TalonFXArmConstants.FIRST_JOINT_FEEDFORWARD.calculate(
                Units.degreesToRadians(position),
                Units.degreesToRadians(velocity)
        );
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

        final PositionVoltage positionVoltage = new PositionVoltage(
                Conversions.degreesToRevolutions(position),
                TalonFXArmConstants.USE_FOC,
                feedforward,
                0,
                false
        );

        secondJointMotor.setControl(positionVoltage);
    }

    @Override
    public void setNeutralMode(boolean brake) {
        final NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        final MotorOutputConfigs
                firstMotorConfig = new MotorOutputConfigs(),
                secondMotorConfig = new MotorOutputConfigs();

        firstJointMotor.getConfigurator().refresh(firstMotorConfig);
        firstMotorConfig.NeutralMode = mode;
        firstJointMotor.getConfigurator().apply(firstMotorConfig);

        secondJointMotor.getConfigurator().refresh(secondMotorConfig);
        secondMotorConfig.NeutralMode = mode;
        secondJointMotor.getConfigurator().apply(secondMotorConfig);
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

    private double secondJointValueToSystemDegrees(double value) {
        final double systemRevolutions = Conversions.systemToMotor(value, ArmConstants.SECOND_JOINT_GEAR_RATIO);

        return Conversions.revolutionsToDegrees(systemRevolutions);
    }
}

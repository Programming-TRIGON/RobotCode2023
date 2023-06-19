package frc.trigon.robot.subsystems.gripper.talonfxgripper;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.gripper.GripperIO;
import frc.trigon.robot.subsystems.gripper.GripperInputsAutoLogged;

public class TalonFXGripperIO extends GripperIO {
    private final TalonFX motor = TalonFXGripperConstants.MOTOR;

    @Override
    public void updateInputs(GripperInputsAutoLogged inputs) {
        inputs.velocityRevolutionsPerSecond = motor.getVelocity().getValue();
        inputs.statorCurrent = motor.getStatorCurrent().getValue();
    }

    @Override
    public void setTargetPower(double power) {
        motor.set(power);
    }

    @Override
    public void configHoldTrigger(Runnable callback) {
        TalonFXGripperConstants.HOLD_TRIGGER_CONFIG.setup(callback);
    }
}

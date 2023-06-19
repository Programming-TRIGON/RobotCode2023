package frc.trigon.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public class GripperIO {
    @AutoLog
    protected static class GripperInputs {
        public double statorCurrent = 0;
        public double velocityRevolutionsPerSecond = 0;
    }

    /**
     * Updates the inputs of the gripper.
     *
     * @param inputs the inputs class to update
     */
    protected void updateInputs(GripperInputsAutoLogged inputs) {
    }

    /**
     * Sets the power of the gripper motor.
     *
     * @param power the power from -1 to 1
     */
    protected void setTargetPower(double power) {
    }

    /**
     * configures the hold trigger.
     *
     * @param callback the callback to run when the hold trigger is triggered
     */
    protected void configHoldTrigger(Runnable callback) {
    }
}

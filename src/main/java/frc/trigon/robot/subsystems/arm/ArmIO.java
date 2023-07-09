package frc.trigon.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
    @AutoLog
    protected static class ArmInputs {
        public double firstJointPositionDegrees = 0;
        public double firstJointVelocityDegreesPerSecond = 0;
        public double firstJointStatorCurrent = 0;
        public double firstJointSupplyCurrent = 0;
        public double firstJointAppliedVoltage = 0;
        public double firstJointClosedLoopOutput = 0;

        public double secondJointPositionDegrees = 0;
        public double secondJointVelocityDegreesPerSecond = 0;
        public double secondJointStatorCurrent = 0;
        public double secondJointSupplyCurrent = 0;
        public double secondJointAppliedVoltage = 0;
        public double secondJointClosedLoopOutput = 0;
    }

    /**
     * Updates the inputs of the arm.
     *
     * @param inputs the inputs class to update
     */
    protected void updateInputs(ArmInputsAutoLogged inputs) {
    }

    /**
     * Sets the position of the first joint.
     *
     * @param position the position in degrees
     * @param velocity the velocity in degrees per second
     */
    protected void setTargetFirstJointPosition(double position, double velocity) {
    }

    /**
     * Sets the position of the second joint.
     *
     * @param position the position in degrees
     * @param velocity the velocity in degrees per second
     */
    protected void setTargetSecondJointPosition(double position, double velocity) {
    }

    /**
     * Sets up the arm limits.
     *
     * @param firstJointCallback  the callback to run when the first arm limit is reached
     * @param secondJointCallback the callback to run when the second arm limit is reached
     */
    protected void setupLimits(Runnable firstJointCallback, Runnable secondJointCallback) {
    }

    /**
     * Sets whether the arm is in brake mode or not.
     *
     * @param brake whether the arm is in brake mode or not
     */
    protected void setNeutralMode(boolean brake) {
    }

    /**
     * Stops the first joint.
     */
    protected void stopFirstJoint() {
    }

    /**
     * Stops the second joint.
     */
    protected void stopSecondJoint() {
    }

    /**
     * Stops the arm.
     */
    void stop() {
        stopFirstJoint();
        stopSecondJoint();
    }
}

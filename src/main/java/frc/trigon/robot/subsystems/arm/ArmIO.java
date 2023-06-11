package frc.trigon.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmInputs {
        public double firstJointPositionDegrees = 0;
        public double firstJointVelocityDegreesPerSecond = 0;
        public double firstJointStatorCurrent = 0;
        public double firstJointSupplyCurrent = 0;
        public double firstJointAppliedVoltage = 0;

        public double secondJointPositionDegrees = 0;
        public double secondJointVelocityDegreesPerSecond = 0;
        public double secondJointStatorCurrent = 0;
        public double secondJointSupplyCurrent = 0;
        public double secondJointAppliedVoltage = 0;
    }

    /**
     * Updates the inputs of the arm.
     *
     * @param inputs the inputs class to update
     */
    default void updateInputs(ArmInputsAutoLogged inputs) {
    }

    /**
     * Sets the position of the first joint.
     *
     * @param position the position in degrees
     * @param velocity the velocity in degrees per second
     */
    default void setFirstJointPosition(double position, double velocity) {
    }

    /**
     * Sets the position of the second joint.
     *
     * @param position the position in degrees
     * @param velocity the velocity in degrees per second
     */
    default void setSecondJointPosition(double position, double velocity) {
    }

    default void setupLimits(Runnable firstJointCallback, Runnable secondJointCallback) {
    }

    /**
     * Sets whether the arm is in brake mode or not.
     *
     * @param brake whether the arm is in brake mode or not
     */
    default void setNeutralMode(boolean brake) {
    }

    /**
     * Stops the first joint.
     */
    default void stopFirstJoint() {
    }

    /**
     * Stops the second joint.
     */
    default void stopSecondJoint() {
    }

    /**
     * Stops the arm.
     */
    default void stop() {
        stopFirstJoint();
        stopSecondJoint();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot.utilities;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * This class represents a controller that either is on full power or off.
 * It is useful for controlling mechanisms that have a large amount of freedom.
 * When starting the controller, it will turn on the motor until the setpoint is reached within the tolerance.
 * Once the setpoint is reached, the controller will turn off the motor,
 * and will not turn it back on until the error is greater than the trigger threshold.
 */
public class LargeFreedomBangBangController implements Sendable{
    private double
            tolerance,
            triggerThreshold,
            setpoint,
            measurement;

    private boolean beenAtSetpoint;

    /**
     * Constructs a new LargeFreedomBangBangController.
     *
     * @param tolerance        the tolerance for reaching the setpoint
     * @param triggerThreshold the threshold at which the controller will trigger again
     */
    public LargeFreedomBangBangController(double tolerance, double triggerThreshold) {
        setTolerance(tolerance);
        setTriggerThreshold(triggerThreshold);
    }

    /**
     * Sets the trigger threshold.
     * @param triggerThreshold the trigger threshold
     */
    public void setTriggerThreshold(double triggerThreshold) {
        this.triggerThreshold = triggerThreshold;
    }

    /**
     * @return the trigger threshold
     */
    public double getTriggerThreshold() {
        return triggerThreshold;
    }

    /**
     * Sets the setpoint for the bang-bang controller.
     *
     * @param setpoint the target setpoint
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        beenAtSetpoint = false;
    }

    /**
     * Returns the current setpoint.
     *
     * @return the current setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * @return whether the controller is at the setpoint,
     * or has been at the setpoint and the error is less than the trigger threshold
     */
    public boolean atSetpoint() {
        return beenAtSetpoint =
                (getAbsError() < triggerThreshold && beenAtSetpoint) ||
                        getAbsError() < tolerance;
    }

    private double getAbsError() {
        return Math.abs(getError());
    }

    /**
     * Sets the acceptable position error which is tolerable.
     *
     * @param tolerance the acceptable position error
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Returns the current tolerance of the controller.
     *
     * @return the current tolerance
     */
    public double getTolerance() {
        return tolerance;
    }

    /**
     * Returns the current error.
     *
     * @return the current error
     */
    public double getError() {
        return setpoint - measurement;
    }

    /**
     * Returns the calculated control output.
     * The output of this function should be multiplied by a power value to get the final motor output.
     *
     * @param measurement The most recent measurement of the process variable.
     * @param setpoint    The setpoint for the process variable.
     * @return The calculated motor output (-1, 0, or 1).
     */
    public double calculate(double measurement, double setpoint) {
        this.measurement = measurement;
        this.setpoint = setpoint;

        if(atSetpoint())
            return 0;
        return Math.signum(getError());
    }

    /**
     * Returns the calculated control output.
     * The output of this function should be multiplied by a power value to get the final motor output.
     *
     * @param measurement The most recent measurement of the process variable.
     * @return The calculated motor output (-1, 0, or 1).
     */
    public double calculate(double measurement) {
        return calculate(measurement, setpoint);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("error", this::getError, null);
        builder.addDoubleProperty("absError", this::getAbsError, null);
        builder.addDoubleProperty("tolerance", this::getTolerance, this::setTolerance);
        builder.addDoubleProperty("triggerThreshold", this::getTriggerThreshold, this::setTriggerThreshold);
        builder.addBooleanProperty("atSetpoint", this::atSetpoint, null);
        builder.addBooleanProperty("beenAtSetpoint", () -> beenAtSetpoint, null);
    }
}

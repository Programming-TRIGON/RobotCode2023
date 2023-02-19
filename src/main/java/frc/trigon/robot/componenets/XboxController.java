package frc.trigon.robot.componenets;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController extends CommandXboxController {
    private int exponent = 1;
    private double deadband = 0;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public XboxController(int port) {
        super(port);
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port     the port index on the Driver Station that the controller is plugged into
     * @param exponent how much to exponentiate the raw values by
     * @param deadband the deadband for the controller
     */
    public XboxController(int port, int exponent, double deadband) {
        this(port);
        this.exponent = exponent;
        this.deadband = deadband;
    }

    /**
     * Sets the exponent for the controller, which will exponentiate the raw values by the exponent.
     *
     * @param exponent the exponent
     */
    public void setExponent(int exponent) {
        this.exponent = exponent;
    }

    /**
     * Sets the deadband for the controller, which will ignore any values within the deadband.
     *
     * @param deadband the deadband, between 0 and 1
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    @Override
    public double getLeftX() {
        return calculateValue(super.getLeftX());
    }

    @Override
    public double getRightX() {
        return calculateValue(super.getRightX());
    }

    @Override
    public double getLeftY() {
        return calculateValue(super.getLeftY());
    }

    @Override
    public double getRightY() {
        return calculateValue(super.getRightY());
    }

    private double calculateValue(double value) {
        final double exponentiatedValue = Math.pow(value, exponent);
        value = Math.abs(exponentiatedValue) * Math.signum(value);
        if (Math.abs(value) < deadband)
            return 0;
        return value;
    }
}

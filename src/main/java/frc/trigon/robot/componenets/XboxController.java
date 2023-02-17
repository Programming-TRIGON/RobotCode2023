package frc.trigon.robot.componenets;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController extends CommandXboxController {
    private boolean square = false;
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
     * @param square   whether to square the input values
     * @param deadband the deadband for the controller
     */
    public XboxController(int port, boolean square, double deadband) {
        super(port);
        this.square = square;
        this.deadband = deadband;
    }

    /**
     * Determines whether the controller is in square mode,
     * which will take the raw value and square it for added precision in the lower values.
     *
     * @param square whether to square the raw values
     */
    public void setSquare(boolean square) {
        this.square = square;
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
        if (!square)
            return value;
        value = Math.pow(value, 2) * Math.signum(value);
        if(Math.abs(value) < deadband)
            return 0;
        return value;
    }
}

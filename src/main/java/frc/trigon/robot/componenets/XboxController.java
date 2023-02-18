package frc.trigon.robot.componenets;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController extends CommandXboxController {
    private boolean square = false;
    private double deadband = 0;
    private edu.wpi.first.wpilibj.XboxController.Axis shiftModeAxis = null;

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
        this(port);
        this.square = square;
        this.deadband = deadband;
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port          the port index on the Driver Station that the controller is plugged into
     * @param square        whether to square the input values
     * @param deadband      the deadband for the controller
     * @param shiftModeAxis the axis to use for shift mode
     */
    public XboxController(int port, boolean square, double deadband, edu.wpi.first.wpilibj.XboxController.Axis shiftModeAxis) {
        this(port, square, deadband);
        this.shiftModeAxis = shiftModeAxis;
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

    /**
     * Sets the axis to use for shift mode.
     *
     * @param shiftModeAxis the axis to use for shift mode
     */
    public void setShiftModeAxis(edu.wpi.first.wpilibj.XboxController.Axis shiftModeAxis) {
        this.shiftModeAxis = shiftModeAxis;
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

    private double getShitModeAxisValue() {
        if (shiftModeAxis == null)
            return 1;

        final double rawValue = super.getRawAxis(shiftModeAxis.value);

        return calculateShiftModeAxisValue(rawValue);
    }

    private double calculateShiftModeAxisValue(double value) {
        if (!square)
            return value;

        return 1 + Math.pow(value, 2);
    }

    private double calculateValue(double value) {
        if (!square)
            return value;
        value = Math.pow(value, 2) * Math.signum(value);
        if (Math.abs(value) < deadband)
            return 0;
        return value * getShitModeAxisValue();
    }
}

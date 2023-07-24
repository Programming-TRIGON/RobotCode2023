package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIO {
    private final SwerveModuleInputsAutoLogged swerveModuleInputs = new SwerveModuleInputsAutoLogged();
    private final String name;
    private boolean driveMotorClosedLoop = false;
    private double targetVelocity, targetAngle;

    public SwerveModuleIO(String name) {
        this.name = name;
    }

    /**
     * Sets whether the drive motor should be in closed loop control, or in open loop control.
     *
     * @param closedLoop true if the drive motor should be in closed loop control, false if it should be in open loop control
     */
    public void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }

    /**
     * This method should be called periodically to update the inputs and network tables of the module.
     */
    public void periodic() {
        updateInputs(swerveModuleInputs);
        Logger.getInstance().processInputs(getLoggingPath(), swerveModuleInputs);

        updateNetworkTables();
    }

    /**
     * Sets the target state for the module.
     *
     * @param targetState the target state
     */
    public void setTargetState(SwerveModuleState targetState) {
        targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond);
        targetAngle = targetState.angle.getDegrees();
        targetVelocity = targetState.speedMetersPerSecond;
    }

    /**
     * @return the current position of the module
     */
    SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(swerveModuleInputs.driveDistanceMeters, getCurrentAngle());
    }

    /**
     * @return the current state of the module
     */
    SwerveModuleState getCurrentState() {
        return new SwerveModuleState(swerveModuleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    /**
     * @return the module's current drive distance in meters
     */
    double getDriveDistance() {
        return swerveModuleInputs.driveDistanceMeters;
    }

    private void updateNetworkTables() {
        Logger.getInstance().recordOutput(getLoggingPath() + "/targetAngle", targetAngle);
        Logger.getInstance().recordOutput(getLoggingPath() + "/targetVelocity", targetVelocity);
    }

    /**
     * Sets the target velocity for the module. In meters per second.
     *
     * @param velocity the target velocity
     */
    private void setTargetVelocity(double velocity) {
        if (driveMotorClosedLoop)
            setTargetClosedLoopVelocity(velocity);
        else
            setTargetOpenLoopVelocity(velocity);
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(swerveModuleInputs.steerAngleDegrees);
    }

    protected String getLoggingPath() {
        return "Swerve/" + name + "/";
    }

    /**
     * Updates the inputs of the swerve module.
     *
     * @param inputs the inputs class to update
     */
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    }

    /**
     * Sets the target open loop velocity.
     *
     * @param velocity the velocity in meters per second
     */
    protected void setTargetOpenLoopVelocity(double velocity) {
    }

    /**
     * Sets the target closed loop velocity.
     *
     * @param velocity the velocity in meters per second
     */
    protected void setTargetClosedLoopVelocity(double velocity) {
    }

    /**
     * Sets the target angle of the module.
     *
     * @param angle the target angle
     */
    protected void setTargetAngle(Rotation2d angle) {
    }

    /**
     * Stops the module.
     */
    protected void stop() {
    }

    /**
     * Sets whether the drive motor should brake or coast
     *
     * @param brake true if the drive motor should brake, false if it should coast
     */
    protected void setBrake(boolean brake) {
    }

    @AutoLog
    public static class SwerveModuleInputs {
        public double steerAngleDegrees = 0;
        public double steerAppliedVoltage = 0;
        public double driveVelocityRevolutionsPerSecond = 0;
        public double driveVelocityMetersPerSecond = 0;
        public double driveDistanceMeters = 0;
        public double drivePositionRevolutions = 0;
        public double driveAppliedVoltage = 0;
    }
}

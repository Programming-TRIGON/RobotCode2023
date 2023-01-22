package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.trigon.robot.utilities.Conversions;

public class SwerveModule implements Sendable {
    private final WPI_TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final SparkMaxAbsoluteEncoder steerEncoder;
    private final double encoderOffset;

    private SwerveModuleState targetState = new SwerveModuleState();
    private boolean driveMotorClosedLoop = false;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        this.driveMotor = moduleConstants.driveMotor;
        this.steerMotor = moduleConstants.steerMotor;
        this.steerEncoder = this.steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.encoderOffset = moduleConstants.encoderOffset;
    }

    /**
     * @return the target state of the module
     */
    SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Sets the target state for the module.
     *
     * @param targetState the target state
     */
    void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState = optimizeState(targetState);
        setTargetAngle(targetState.angle.getDegrees());
        setTargetVelocity(targetState.speedMetersPerSecond);
    }

    /**
     * @return the current state of the module
     */
    SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentVelocity(), getCurrentAngle());
    }

    /**
     * Sets whether the drive motor should brake or coast
     *
     * @param brake true if the drive motor should brake, false if it should coast
     */
    void setBrake(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Stops the module from moving.
     */
    void stop() {
        driveMotor.disable();
        steerMotor.disable();
    }

    /**
     * @return the current position of the swerve module
     */
    SwerveModulePosition getCurrentModulePosition() {
        return new SwerveModulePosition(getCurrentMotorPositionInMeters(), getCurrentAngle());
    }

    /**
     * Sets whether the drive motor should be in closed loop control, or in open loop control.
     *
     * @param closedLoop true if the drive motor should be in closed loop control, false if it should be in open loop control
     */
    void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }

    private double getCurrentMotorPositionInMeters() {
        double
                motorRotations = driveMotor.getSelectedSensorPosition(),
                systemRotations = Conversions.motorToSystem(motorRotations, SwerveModuleConstants.DRIVE_GEAR_RATIO);

        return Conversions.revolutionsToDistance(systemRotations, SwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private void setTargetVelocity(double velocity) {
        if (driveMotorClosedLoop) {
            setTargetClosedLoopVelocity(velocity);
        } else {
            setTargetOpenLoopVelocity(velocity);
        }
    }

    private void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocity = Conversions.systemToMotor(velocity, SwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double feedForward = SwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(driveMotorVelocity);

        driveMotor.set(
                ControlMode.Velocity, driveMotorVelocity,
                DemandType.ArbitraryFeedForward, feedForward
        );
    }

    private void setTargetOpenLoopVelocity(double velocity) {
        double power = velocity / SwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;

        driveMotor.set(power);
    }

    private void setTargetAngle(double targetAngle) {
        double targetAngleRevolutions = Conversions.degreesToRevolutions(targetAngle);
        double offsettedRevolutions = Conversions.offsetWrite(targetAngleRevolutions, encoderOffset);

        steerMotor.getPIDController().setReference(offsettedRevolutions, ControlType.kPosition);
    }

    /**
     * @return the module's current velocity in mps
     */
    private double getCurrentVelocity() {
        double
                motorTicksPer100Ms = driveMotor.getSelectedSensorVelocity(),
                motorRevolutionsPer100Ms = Conversions.falconTicksToRevolutions(motorTicksPer100Ms);
        double
                motorRps = Conversions.perHundredMsToPerSecond(motorRevolutionsPer100Ms),
                wheelRps = Conversions.motorToSystem(motorRps, SwerveModuleConstants.DRIVE_GEAR_RATIO);

        return Conversions.revolutionsToDistance(wheelRps, SwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(getCurrentDegrees());
    }

    /**
     * @return the module's current angle in degrees
     */
    private double getCurrentDegrees() {
        double
                encoderRotations = steerEncoder.getPosition(),
                offsettedRotations = Conversions.offsetRead(encoderRotations, encoderOffset);

        return Conversions.revolutionsToDegrees(offsettedRotations);
    }

    /**
     * Minimize the change in the heading the target swerve module state would require
     * by potentially reversing the direction the wheel spins. Customized from WPILib's
     * version to include placing in appropriate scope for closed loop control on the motor controller.
     *
     * @param state the target angle for the module
     */
    private SwerveModuleState optimizeState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getCurrentAngle());
        optimized.angle = Rotation2d.fromDegrees(scope(optimized.angle.getDegrees()));

        return optimized;
    }

    /**
     * Returns an angle that's equivalent to the given angle,
     * but is within 180 degrees of the reference angle.
     *
     * @param angle the angle to scope in degrees
     * @return the scoped angle
     */
    private double scope(double angle) {
        double rawCurrentAngle = getCurrentDegrees() % 360;
        double rawTargetAngle = angle % 360;
        double difference = rawTargetAngle - rawCurrentAngle;
        if (difference < -180)
            difference += 360;
        else if (difference > 180)
            difference -= 360;

        return difference + getCurrentDegrees();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", this::getCurrentDegrees, null);
        builder.addDoubleProperty("velocity", this::getCurrentVelocity, null);
        builder.addDoubleProperty("targetAngle", () -> targetState.angle.getDegrees(), this::setTargetAngle);
        builder.addDoubleProperty("targetVelocity", () -> targetState.speedMetersPerSecond, this::setTargetOpenLoopVelocity);
        builder.addDoubleProperty(
                "rawAngleDeg", () -> Conversions.revolutionsToDegrees(steerEncoder.getPosition()), null);
    }
}


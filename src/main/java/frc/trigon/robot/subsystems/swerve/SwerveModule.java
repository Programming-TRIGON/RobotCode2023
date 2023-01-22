package frc.trigon.robot.subsystems.swerve;

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
    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        this.driveMotor = moduleConstants.driveMotor;
        this.steerMotor = moduleConstants.steerMotor;
        this.steerEncoder = this.steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    }

    /**
     * @return the target state of the module
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Sets the target state for the module.
     *
     * @param targetState the target state
     */
    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState = SwerveModuleState.optimize(targetState, Rotation2d.fromDegrees(getCurrentAngle()));
        setTargetAngle(targetState.angle.getDegrees());
        setTargetVelocity(targetState.speedMetersPerSecond);
    }

    /**
     * @return the current state of the module
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentVelocity(), Rotation2d.fromDegrees(getCurrentAngle()));
    }

    private void setTargetVelocity(double velocity) {
        double power = velocity / SwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        driveMotor.set(power);
    }

    private void setTargetAngle(double targetAngle) {
        steerMotor.getPIDController().setReference(targetAngle, ControlType.kPosition);
    }

    /**
     * @return the distance the module has traveled in meters
     */
    private double getDriveDistance() {
        double ticks = driveMotor.getSelectedSensorPosition();
        double motorRevolutions = Conversions.falconTicksToRevolutions(ticks);
        double wheelRevolutions = Conversions.motorToSystem(motorRevolutions, SwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRevolutions, SwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    /**
     * @return the module's current velocity in mps
     */
    private double getCurrentVelocity() {
        double motorTicksPer100Ms = driveMotor.getSelectedSensorVelocity();
        double motorRevolutionsPer100Ms = Conversions.falconTicksToRevolutions(motorTicksPer100Ms);
        double motorRps = Conversions.perHundredMsToPerSecond(motorRevolutionsPer100Ms);
        double wheelRps = Conversions.motorToSystem(motorRps, SwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRps, SwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    /**
     * @return the module's current angle in degrees
     */
    private double getCurrentAngle() {
        return steerEncoder.getPosition();
    }

    /**
     * Sets whether the drive motor should brake or coast
     *
     * @param brake true if the drive motor should brake, false if it should coast
     */
    public void setBrake(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Stops the module from moving.
     */
    public void stop() {
        driveMotor.disable();
        steerMotor.disable();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", this::getCurrentAngle, null);
        builder.addDoubleProperty("velocity", this::getCurrentVelocity, null);
        builder.addDoubleProperty("targetAngle", () -> targetState.angle.getDegrees(), this::setTargetAngle);
        builder.addDoubleProperty("targetVelocity", () -> targetState.speedMetersPerSecond, this::setTargetVelocity);
        builder.addDoubleProperty(
                "rawAngleDeg", () -> Conversions.revolutionsToDegrees(steerEncoder.getPosition()), null);
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromDegrees(getCurrentAngle()));
    }
}


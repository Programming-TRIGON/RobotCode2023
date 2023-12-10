package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;

    public TrihardSwerveModuleIO(TrihardSwerveModuleConstants.TrihardSwerveModules module) {
        super(module.name());
        final TrihardSwerveModuleConstants moduleConstants = module.swerveModuleConstants;

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.steerAppliedVoltage = steerMotor.getSupplyVoltage().getValue();
        inputs.drivePositionRevolutions = driveMotor.getPosition().getValue();
        inputs.driveDistanceMeters = driveMotorValueToDistance(inputs.drivePositionRevolutions);
        inputs.driveVelocityMetersPerSecond = driveMotorValueToDistance(inputs.driveVelocityRevolutionsPerSecond);
        inputs.driveVelocityRevolutionsPerSecond = driveMotor.getVelocity().getValue();
        inputs.driveAppliedVoltage = driveMotor.getSupplyVoltage().getValue();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        final double power = velocity / TrihardSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        final VoltageOut request = new VoltageOut(
                power * TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION,
                driveMotor.getDeviceID() == 1 || driveMotor.getDeviceID() == 2,
                false
        );
        driveMotor.setControl(request);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocityMeters = Conversions.systemToMotor(velocity, TrihardSwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double driverMotorVelocityRevolutions = Conversions.distanceToRevolutions(driveMotorVelocityMeters, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double feedforward = TrihardSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(velocity);
        final VelocityVoltage velocityVoltage = new VelocityVoltage(
                driverMotorVelocityRevolutions, TrihardSwerveModuleConstants.DRIVE_MOTOR_FOC,
                feedforward, 0, false
        );

        driveMotor.setControl(velocityVoltage);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        final double scopedAngle = scope(angle);
        final double motorAngle = Conversions.systemToMotor(scopedAngle, TrihardSwerveModuleConstants.STEER_GEAR_RATIO);
        final double motorRevolutions = Conversions.degreesToRevolutions(motorAngle);
        final PositionVoltage positionVoltage = new PositionVoltage(motorRevolutions);
        steerMotor.setControl(positionVoltage);
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();

        driveMotor.getConfigurator().refresh(driveMotorOutputConfigs);
        driveMotorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveMotorOutputConfigs);
    }

    private double getAngleDegrees() {
        final double motorRevolutions = steerMotor.getPosition().getValue();
        final double motorDegrees = Conversions.revolutionsToDegrees(motorRevolutions);

        return Conversions.motorToSystem(motorDegrees, TrihardSwerveModuleConstants.STEER_GEAR_RATIO);
    }

    private double driveMotorValueToDistance(double value) {
        final double systemRevolutions = Conversions.motorToSystem(value, TrihardSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(systemRevolutions, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private double scope(Rotation2d targetRotation2d) {
        double currentDegrees = getAngleDegrees() % 360;
        double targetDegrees = targetRotation2d.getDegrees() % 360;
        double difference = targetDegrees - currentDegrees;
        if (difference < -180)
            difference += 360;
        else if (difference > 180)
            difference -= 360;

        return difference + getAngleDegrees();
    }
}

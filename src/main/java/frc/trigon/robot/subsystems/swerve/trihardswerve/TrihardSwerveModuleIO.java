package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final String name;

    public TrihardSwerveModuleIO(TrihardSwerveModuleConstants.TrihardSwerveModules module) {
        final TrihardSwerveModuleConstants moduleConstants = module.swerveModuleConstants;

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        this.name = module.name();
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        if (!inputs.name.equals(name))
            inputs.name = name;

        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.steerAppliedVoltage = steerMotor.getSupplyVoltage().getValue();
        inputs.drivePositionRevolutions = driveMotor.getPosition().getValue();
        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(inputs.drivePositionRevolutions, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(inputs.driveVelocityRevolutionsPerSecond, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityRevolutionsPerSecond = driveMotor.getVelocity().getValue();
        inputs.driveAppliedVoltage = driveMotor.getSupplyVoltage().getValue();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        final double power = velocity / TrihardSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        driveMotor.set(power);
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
        final PositionVoltage positionVoltage = new PositionVoltage(scope(angle));
        driveMotor.setControl(positionVoltage);
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
package frc.trigon.robot.subsystems.swerve.testingswerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TestingSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final SparkMaxAbsoluteEncoder steerEncoder;

    public TestingSwerveModuleIO(TestingSwerveModuleConstants.TestingSwerveModules module) {
        super(module.name());
        final TestingSwerveModuleConstants moduleConstants = module.swerveModuleConstants;

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        steerEncoder = steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = steerEncoder.getPosition();
        inputs.steerAppliedVoltage = steerMotor.getBusVoltage();
        inputs.drivePositionRevolutions = driveMotor.getPosition().getValue();
        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(inputs.drivePositionRevolutions, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(inputs.driveVelocityRevolutionsPerSecond, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityRevolutionsPerSecond = driveMotor.getVelocity().getValue();
        inputs.driveAppliedVoltage = driveMotor.getSupplyVoltage().getValue();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        double power = velocity / TestingSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        driveMotor.set(power);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocityMeters = Conversions.systemToMotor(velocity, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double driverMotorVelocityRevolutions = Conversions.distanceToRevolutions(driveMotorVelocityMeters, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double feedforward = TestingSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(velocity);
        final VelocityVoltage velocityVoltage = new VelocityVoltage(
                driverMotorVelocityRevolutions, TestingSwerveModuleConstants.DRIVE_MOTOR_FOC,
                feedforward, 0, false
        );

        driveMotor.setControl(velocityVoltage);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.getPIDController().setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final MotorOutputConfigs driveMotorOutputConfig = new MotorOutputConfigs();

        driveMotor.getConfigurator().refresh(driveMotorOutputConfig);
        driveMotorOutputConfig.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveMotorOutputConfig);
    }
}

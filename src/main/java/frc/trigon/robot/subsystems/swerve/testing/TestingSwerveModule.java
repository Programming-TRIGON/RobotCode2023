package frc.trigon.robot.subsystems.swerve.testing;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.trigon.robot.subsystems.swerve.SwerveModule;
import frc.trigon.robot.utilities.Conversions;

public class TestingSwerveModule extends SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final SparkMaxAbsoluteEncoder steerEncoder;
    private final String moduleName;
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    TestingSwerveModule(TestingSwerveModuleConstants.TestingSwerveModules swerveModule) {
        final TestingSwerveModuleConstants moduleConstants = swerveModule.swerveModuleConstants;

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
        steerEncoder = steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.moduleName = swerveModule.name();
    }

    @Override
    protected Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    protected void setBrake(boolean brake) {
        final MotorOutputConfigs driveMotorOutputConfig = new MotorOutputConfigs();

        driveMotor.getConfigurator().refresh(driveMotorOutputConfig);
        driveMotorOutputConfig.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveMotorOutputConfig);
    }

    @Override
    protected void stop() {
        driveMotor.disable();
        steerMotor.disable();
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocityMeters = Conversions.systemToMotor(velocity, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double driverMotorVelocityRevolutions = Conversions.distanceToRevolutions(driveMotorVelocityMeters, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double feedForward = TestingSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(velocity);

        driveMotor.setControl(
                // TODO: check this
                new VelocityVoltage(driverMotorVelocityRevolutions, TestingSwerveModuleConstants.DRIVE_MOTOR_FOC, feedForward, 0, false)
        );
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        double power = velocity / TestingSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        driveMotor.set(power);
    }

    @Override
    protected String getModuleName() {
        return moduleName;
    }

    @Override
    protected void setTargetAngle(Rotation2d rotation2d) {
        targetAngle = rotation2d;
        steerMotor.getPIDController().setReference(rotation2d.getDegrees(), ControlType.kPosition);
    }

    @Override
    protected double getDriveDistance() {
        double position = driveMotor.getPosition().getValue();
        double wheelRevolutions = Conversions.motorToSystem(position, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRevolutions, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected SwerveModuleState optimizeState(SwerveModuleState state) {
        return SwerveModuleState.optimize(state, getCurrentAngle());
    }

    @Override
    protected double getCurrentVelocity() {
        double motorRps = driveMotor.getVelocity().getValue();
        double wheelRps = Conversions.motorToSystem(motorRps, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRps, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
}


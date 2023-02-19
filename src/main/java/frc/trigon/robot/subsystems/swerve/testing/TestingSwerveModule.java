package frc.trigon.robot.subsystems.swerve.testing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.trigon.robot.subsystems.swerve.SwerveModule;
import frc.trigon.robot.utilities.Conversions;

public class TestingSwerveModule extends SwerveModule {
    private final WPI_TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final SparkMaxAbsoluteEncoder steerEncoder;
    private final String moduleName;

    TestingSwerveModule(TestingSwerveModuleConstants.TestingSwerveModules swerveModule) {
        final TestingSwerveModuleConstants moduleConstants = swerveModule.swerveModuleConstants;

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
        steerEncoder = steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.moduleName = swerveModule.name();
    }

    @Override
    protected void setBrake(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    protected void stop() {
        driveMotor.disable();
        steerMotor.disable();
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocity = Conversions.systemToMotor(velocity, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double feedForward = TestingSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(driveMotorVelocity);

        driveMotor.set(
                ControlMode.Velocity, driveMotorVelocity,
                DemandType.ArbitraryFeedForward, feedForward
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
        steerMotor.getPIDController().setReference(rotation2d.getDegrees(), ControlType.kPosition);
    }

    @Override
    protected double getDriveDistance() {
        double ticks = driveMotor.getSelectedSensorPosition();
        double motorRevolutions = Conversions.falconTicksToRevolutions(ticks);
        double wheelRevolutions = Conversions.motorToSystem(motorRevolutions, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRevolutions, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected SwerveModuleState optimizeState(SwerveModuleState state) {
        return SwerveModuleState.optimize(state, getCurrentAngle());
    }

    @Override
    protected double getCurrentVelocity() {
        double motorTicksPer100Ms = driveMotor.getSelectedSensorVelocity();
        double motorRevolutionsPer100Ms = Conversions.falconTicksToRevolutions(motorTicksPer100Ms);
        double motorRps = Conversions.perHundredMsToPerSecond(motorRevolutionsPer100Ms);
        double wheelRps = Conversions.motorToSystem(motorRps, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRps, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
}


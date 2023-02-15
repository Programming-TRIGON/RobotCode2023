package frc.trigon.robot.subsystems.swerve.trihard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.trigon.robot.subsystems.swerve.SwerveModule;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModule extends SwerveModule {
    private final WPI_TalonFX driveMotor, steerMotor;
    private final String name;

    TrihardSwerveModule(TrihardSwerveModuleConstants.TrihardSwerveModules swerveModule) {
        final TrihardSwerveModuleConstants moduleConstants = swerveModule.swerveModuleConstants;

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
        name = swerveModule.name();
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
    protected Rotation2d getCurrentAngle() {
        final double
                currentSteerMotorDegrees = Conversions.falconTicksToDegrees(steerMotor.getSelectedSensorPosition()),
                currentSystemDegrees = Conversions.motorToSystem(currentSteerMotorDegrees, TrihardSwerveModuleConstants.STEER_GEAR_RATIO);

        return Rotation2d.fromDegrees(currentSystemDegrees);
    }

    @Override
    protected double getCurrentVelocity() {
        double motorTicksPer100Ms = driveMotor.getSelectedSensorVelocity();
        double motorRevolutionsPer100Ms = Conversions.falconTicksToRevolutions(motorTicksPer100Ms);
        double motorRps = Conversions.perHundredMsToPerSecond(motorRevolutionsPer100Ms);
        double wheelRps = Conversions.motorToSystem(motorRps, TrihardSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRps, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected double getDriveDistance() {
        double ticks = driveMotor.getSelectedSensorPosition();
        double motorRevolutions = Conversions.falconTicksToRevolutions(ticks);
        double wheelRevolutions = Conversions.motorToSystem(motorRevolutions, TrihardSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRevolutions, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected void setTargetAngle(Rotation2d rotation2d) {
        final double
                targetSteerMotorPosition = Conversions.degreesToFalconTicks(rotation2d.getDegrees()),
                targetSystemPosition = Conversions.motorToSystem(targetSteerMotorPosition, TrihardSwerveModuleConstants.STEER_GEAR_RATIO);

        steerMotor.set(ControlMode.Position, targetSystemPosition);
    }

    @Override
    protected SwerveModuleState optimizeState(SwerveModuleState state) {
        final SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getCurrentAngle());
        optimizedState.angle = scope(optimizedState.angle);

        return optimizedState;
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocity = Conversions.systemToMotor(velocity, TrihardSwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double feedForward = TrihardSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(driveMotorVelocity);

        driveMotor.set(
                ControlMode.Velocity, driveMotorVelocity,
                DemandType.ArbitraryFeedForward, feedForward
        );
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        double power = velocity / TrihardSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        driveMotor.set(power);
    }

    @Override
    protected String getName() {
        return name;
    }

    private Rotation2d scope(Rotation2d targetRotation2d) {
        double rawCurrentDegrees = getCurrentAngle().getDegrees() % 360;
        double rawTargetDegrees = targetRotation2d.getDegrees() % 360;
        double difference = rawTargetDegrees - rawCurrentDegrees;
        if (difference < -180)
            difference += 360;
        else if (difference > 180)
            difference -= 360;

        return Rotation2d.fromDegrees(difference).plus(getCurrentAngle());
    }
}

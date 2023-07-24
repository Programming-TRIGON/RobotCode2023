package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationSwerveModuleIO extends SwerveModuleIO {
    private final DCMotorSim driveMotor, steerMotor;
    private double driveAppliedVoltage, steerAppliedVoltage;
    private SwerveModuleInputsAutoLogged lastInputs = new SwerveModuleInputsAutoLogged();

    public SimulationSwerveModuleIO(SimulationSwerveModuleConstants.SimulationSwerveModules module) {
        super(module.name());
        final SimulationSwerveModuleConstants moduleConstants = module.swerveModuleConstants;

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        steerMotor.update(OperatorConstants.PERIODIC_TIME_SECONDS);
        driveMotor.update(OperatorConstants.PERIODIC_TIME_SECONDS);

        inputs.steerAngleDegrees = Conversions.revolutionsToDegrees(steerMotor.getAngularPositionRotations());
        inputs.steerAppliedVoltage = steerAppliedVoltage;
        inputs.drivePositionRevolutions = driveMotor.getAngularPositionRotations();
        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(inputs.drivePositionRevolutions, SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityRevolutionsPerSecond = Units.radiansToRotations(driveMotor.getAngularVelocityRadPerSec());
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(inputs.driveVelocityRevolutionsPerSecond, SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveAppliedVoltage = driveAppliedVoltage;

        lastInputs = inputs;
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        final double power = velocity / SimulationSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND / 1.3;
        final double voltage = power * SimulationSwerveModuleConstants.MAX_MOTOR_VOLTAGE;
        setDriveVoltage(voltage);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        setTargetOpenLoopVelocity(velocity);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        final double pidOutput = SimulationSwerveModuleConstants.STEER_MOTOR_PID_CONTROLLER.calculate(
                MathUtil.inputModulus(lastInputs.steerAngleDegrees, 0, 360),
                MathUtil.inputModulus(scope(angle), 0, 360)
        );

        setSteerVoltage(pidOutput);
    }

    @Override
    protected void stop() {
        setDriveVoltage(0);
        setSteerVoltage(0);
    }

    private double voltageToMaxedVoltage(double voltage) {
        return MathUtil.clamp(
                voltage,
                -SimulationSwerveModuleConstants.MAX_MOTOR_VOLTAGE,
                SimulationSwerveModuleConstants.MAX_MOTOR_VOLTAGE
        );
    }

    private void setDriveVoltage(double voltage) {
        driveAppliedVoltage = voltageToMaxedVoltage(voltage);
        driveMotor.setInputVoltage(driveAppliedVoltage);
    }

    private void setSteerVoltage(double voltage) {
        steerAppliedVoltage = voltageToMaxedVoltage(voltage);
        steerMotor.setInputVoltage(steerAppliedVoltage);
    }

    private double scope(Rotation2d targetRotation2d) {
        double currentDegrees = lastInputs.steerAngleDegrees % 360;
        double targetDegrees = targetRotation2d.getDegrees() % 360;
        double difference = targetDegrees - currentDegrees;
        if (difference < -180)
            difference += 360;
        else if (difference > 180)
            difference -= 360;

        return difference + lastInputs.steerAngleDegrees;
    }
}

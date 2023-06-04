package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.subsystems.LoggableSubsystemBase;
import frc.trigon.robot.utilities.Conversions;
import io.github.oblarg.oblog.annotations.Log;

import static frc.trigon.robot.subsystems.arm.ArmConstants.ArmStates;

public class Arm extends LoggableSubsystemBase {
    private static final Arm INSTANCE = new Arm();
    private final TalonFX
            firstMotor = ArmConstants.FIRST_JOINT_MASTER_MOTOR,
            secondMotor = ArmConstants.SECOND_JOINT_MOTOR;
    private final ArmFeedforward
            firstMotorFeedforward = ArmConstants.FIRST_JOINT_FEEDFORWARD,
            secondMotorFeedforward = ArmConstants.SECOND_JOINT_FEEDFORWARD;
    private TrapezoidProfile firstMotorProfile, secondMotorProfile;
    private double firstMotorProfileLastSetTime, secondMotorProfileLastSetTime;
    private String firstArmToMove = "";
    private double lastFirstMotorSpeedFactor, lastSecondMotorSpeedFactor;

    private Arm() {
        setCurrentLimits();
    }

    public boolean atGoal() {
        boolean
                firstMotorAtGoal = Math.abs(getFirstMotorPosition() - getFirstMotorGoal()) < ArmConstants.FIRST_JOINT_TOLERANCE,
                secondMotorAtGoal = Math.abs(getSecondMotorPosition() - getSecondMotorGoal()) < ArmConstants.SECOND_JOINT_TOLERANCE,
                firstMotorVelocityAtGoal = Math.abs(getFirstMotorVelocity()) < ArmConstants.FIRST_JOINT_VELOCITY_TOLERANCE,
                secondMotorVelocityAtGoal = Math.abs(getSecondMotorVelocity()) < ArmConstants.SECOND_JOINT_VELOCITY_TOLERANCE;
        SmartDashboard.putBoolean("firstMotorAtGoal", firstMotorAtGoal);
        SmartDashboard.putBoolean("secondMotorAtGoal", secondMotorAtGoal);
        SmartDashboard.putBoolean("firstMotorVelocityAtGoal", firstMotorVelocityAtGoal);
        SmartDashboard.putBoolean("secondMotorVelocityAtGoal", secondMotorVelocityAtGoal);
        return firstMotorAtGoal && secondMotorAtGoal && firstMotorVelocityAtGoal && secondMotorVelocityAtGoal;
    }

    public static Arm getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        setTargetMotorPositions();
    }

    /**
     * Constructs a command that sets the target state of the arm.
     *
     * @param state                  the target state
     * @param byOrder                whether to move the arm by order
     * @param firstMotorSpeedFactor  the speed factor of the first motor
     * @param secondMotorSpeedFactor the speed factor of the second motor
     * @return the command
     */
    public CommandBase getGoToStateCommand(ArmStates state, boolean byOrder, double firstMotorSpeedFactor, double secondMotorSpeedFactor) {
        return new StartEndCommand(
                () -> setTargetState(state, byOrder, firstMotorSpeedFactor, secondMotorSpeedFactor),
                () -> {
                },
                this
        );
    }

    /**
     * Constructs a command that sets the target state of the arm.
     *
     * @param state the target state
     * @return the command
     */
    public CommandBase getGoToStateCommand(ArmConstants.ArmStates state) {
        return getGoToStateCommand(state, true, 1, 1);
    }

    /**
     * Constructs a command that sets the target position to the arm.
     *
     * @param firstJointAngle        the angle of the first joint
     * @param secondJointAngle       the angle of the second joint
     * @param byOrder                whether to move the arm by order
     * @param firstJointSpeedFactor  the speed factor of the first motor
     * @param secondJointSpeedFactor the speed factor of the second motor
     * @return the command
     */
    public Command getGoToPositionCommand(double firstJointAngle, double secondJointAngle, boolean byOrder, double firstJointSpeedFactor, double secondJointSpeedFactor) {
        return new StartEndCommand(
                () -> setTargetState(firstJointAngle, secondJointAngle, byOrder, firstJointSpeedFactor, secondJointSpeedFactor),
                () -> {
                },
                this
        );
    }

    /**
     * Constructs a command that sets the target position to the arm.
     *
     * @param firstJointAngle  the angle of the first joint
     * @param secondJointAngle the angle of the second joint
     * @return the command
     */
    public Command getGoToPositionCommand(double firstJointAngle, double secondJointAngle) {
        return getGoToPositionCommand(firstJointAngle, secondJointAngle, true, 1, 1);
    }

    /**
     * Sets whether the arm is in brake mode or not.
     *
     * @param brake whether the arm is in brake mode or not
     */
    public void setNeutralMode(boolean brake) {
        final NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        final MotorOutputConfigs
                firstMotorConfig = new MotorOutputConfigs(),
                secondMotorConfig = new MotorOutputConfigs();

        firstMotor.getConfigurator().refresh(firstMotorConfig);
        firstMotorConfig.NeutralMode = mode;
        firstMotor.getConfigurator().apply(firstMotorConfig);

        secondMotor.getConfigurator().refresh(secondMotorConfig);
        secondMotorConfig.NeutralMode = mode;
        secondMotor.getConfigurator().apply(secondMotorConfig);
    }

    /**
     * Sets the mode of operation during neutral throttle output, for the arm motors.
     */
    public void setNeutralMode() {
        final MotorOutputConfigs
                firstMotorConfig = new MotorOutputConfigs(),
                secondMotorConfig = new MotorOutputConfigs();

        firstMotor.getConfigurator().refresh(firstMotorConfig);
        firstMotorConfig.NeutralMode = ArmConstants.FIRST_JOINT_NEUTRAL_MODE;
        firstMotor.getConfigurator().apply(firstMotorConfig);

        secondMotor.getConfigurator().refresh(secondMotorConfig);
        secondMotorConfig.NeutralMode = ArmConstants.SECOND_JOINT_NEUTRAL_MODE;
        secondMotor.getConfigurator().apply(secondMotorConfig);
    }

    private void setTargetState(double firstMotorPosition, double secondMotorPosition, boolean byOrder, double firstJointSpeedFactor, double secondJointSpeedFactor) {
        lastFirstMotorSpeedFactor = firstJointSpeedFactor;
        lastSecondMotorSpeedFactor = secondJointSpeedFactor;
        generateFirstMotorProfile(firstMotorPosition, firstJointSpeedFactor);
        generateSecondMotorProfile(secondMotorPosition, secondJointSpeedFactor);
        if (byOrder)
            firstArmToMove = getFirstMotorDistanceToGoal() > 0 ? "first" : "second";
        else
            firstArmToMove = "";
    }

    private void setTargetState(ArmStates targetState, boolean byOrder, double firstMotorSpeedFactor, double secondMotorSpeedFactor) {
        setTargetState(targetState.firstMotorPosition, targetState.secondMotorPosition, byOrder, firstMotorSpeedFactor, secondMotorSpeedFactor);
    }

    private void setTargetState(ArmStates targetState) {
        setTargetState(targetState.firstMotorPosition, targetState.secondMotorPosition, true, 1, 1);
    }

    private void setTargetMotorPositions() {
        if (this.getCurrentCommand() == null) {
            firstMotor.disable();
            secondMotor.disable();
        } else {
            setFirstMotorPositionFromProfile();
            setSecondMotorPositionFromProfile();
        }
    }

    private void generateFirstMotorProfile(double position, double speedFactor) {
        firstMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.FIRST_JOINT_CONSTRAINTS, speedFactor),
                new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(getFirstMotorPosition(), getFirstMotorVelocity())
        );
        firstMotorProfileLastSetTime = Timer.getFPGATimestamp();
    }

    private void generateSecondMotorProfile(double position, double speedFactor) {
        secondMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.SECOND_JOINT_CONSTRAINTS, speedFactor),
                new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(getSecondMotorPosition(), getSecondMotorVelocity())
        );
        secondMotorProfileLastSetTime = Timer.getFPGATimestamp();
    }

    private void setFirstMotorPositionFromProfile() {
        if (firstMotorProfile == null) {
            firstMotor.stopMotor();
            return;
        }

        double profileTime = getFirstMotorProfileTime();

        TrapezoidProfile.State targetState = firstMotorProfile.calculate(profileTime);

        boolean goingToHitTheGround = goingToHitTheGround(targetState);
        boolean waitingForOtherJoint = isNotFirstToMove("first") && (isSecondJointOnlyStarting() && !isSecondJointRetracted());
        if (goingToHitTheGround || waitingForOtherJoint) {
            generateFirstMotorProfile(getFirstMotorGoal(), lastFirstMotorSpeedFactor);
            return;
        }

        double feedforward = calculateFeedforward(firstMotorFeedforward, targetState.position, targetState.velocity);
        double targetPosition = Conversions.degreesToMagTicks(targetState.position);

        setTargetPositionWithFeedforward(firstMotor, targetPosition, feedforward);
    }

    private void setCurrentLimits() {
        ArmConstants.FIRST_JOINT_CURRENT_LIMIT_CONFIG.setup(
                () -> {
                    firstMotorProfile = null;
                    DriverStation.reportWarning("Arm first motor current draw is too high!\t" + firstMotor.getStatorCurrent(), false);
                }
        );
        ArmConstants.SECOND_JOINT_CURRENT_LIMIT_CONFIG.setup(
                () -> {
                    secondMotorProfile = null;
                    DriverStation.reportWarning("Arm second motor current draw is too high!\t" + secondMotor.getStatorCurrent(), false);
                }
        );
    }

    private void setSecondMotorPositionFromProfile() {
        if (secondMotorProfile == null) {
            secondMotor.stopMotor();
            return;
        }

        double profileTime = getSecondMotorProfileTime();
        TrapezoidProfile.State targetState = secondMotorProfile.calculate(profileTime);

        boolean goingToHitTheGround = goingToHitTheGround(targetState);
        boolean waitingForOtherJoint = isNotFirstToMove("second") && (isFirstJointOnlyStarting() && isSecondJointRetracted());

        double targetPosition = Conversions.degreesToMagTicks(targetState.position);
        if (goingToHitTheGround || waitingForOtherJoint) {
            generateSecondMotorProfile(getSecondMotorGoal(), lastSecondMotorSpeedFactor);
            secondMotor.stopMotor();
//            targetPosition = Conversions.degreesToMagTicks(getSecondMotorPosition());
        }
        double feedforward = calculateFeedforward(
                secondMotorFeedforward,
                targetState.position + getFirstMotorPosition(),
                targetState.velocity
        );

        setTargetPositionWithFeedforward(secondMotor, targetPosition, feedforward);
        SmartDashboard.putNumber("second motor setpoint", targetState.position);
    }

    private double getFirstArmPercentage() {
        if (firstMotorProfile == null)
            return 1;
        return getFirstMotorProfileTime() / firstMotorProfile.totalTime();
    }

    private double getSecondArmPercentage() {
        if (secondMotorProfile == null)
            return 1;
        return getSecondMotorProfileTime() / secondMotorProfile.totalTime();
    }

    private boolean isFirstJointOnlyStarting() {
        return getFirstArmPercentage() < ArmConstants.RISE_PROFILE_COMPLETION_PERCENTAGE;
    }

    private boolean isSecondJointOnlyStarting() {
        return getSecondArmPercentage() < ArmConstants.DESCEND_PROFILE_COMPLETION_PERCENTAGE;
    }

    private double getFirstMotorProfileTime() {
        return Timer.getFPGATimestamp() - firstMotorProfileLastSetTime;
    }

    private double getSecondMotorProfileTime() {
        return Timer.getFPGATimestamp() - secondMotorProfileLastSetTime;
    }

    private double getFirstMotorGoal() {
        if (firstMotorProfile == null)
            return getFirstMotorPosition();
        return firstMotorProfile.calculate(firstMotorProfile.totalTime()).position;
    }

    private double getSecondMotorGoal() {
        if (secondMotorProfile == null)
            return getSecondMotorPosition();
        return secondMotorProfile.calculate(secondMotorProfile.totalTime()).position;
    }

    private double getFirstMotorDistanceToGoal() {
        return getFirstMotorGoal() - getFirstMotorPosition();
    }

    private double getSecondMotorDistanceToGoal() {
        return getSecondMotorGoal() - getSecondMotorPosition();
    }

    private boolean goingToHitTheGround(TrapezoidProfile.State targetState) {
        return targetState.velocity < 0 && getCurrentEndEffectorLocation().getY() < 20;
    }

    private Translation2d getCurrentEndEffectorLocation() {
        return calculateEndEffectorLocation(getFirstMotorPosition(), getSecondMotorPosition());
    }

    private boolean isNotFirstToMove(String arm) {
        return (!firstArmToMove.equals(arm)) && !firstArmToMove.isBlank();
    }

    private Translation2d calculateEndEffectorLocation(double alpha, double beta) {
        var secondJointLocation =
                new Pose2d(new Translation2d(0, ArmConstants.FIRST_JOINT_HEIGHT), Rotation2d.fromDegrees(alpha)).plus(new Transform2d(new Translation2d(ArmConstants.FIRST_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0)));
        return secondJointLocation.plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(beta))).plus(new Transform2d(new Translation2d(ArmConstants.SECOND_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0))).getTranslation();
    }

    private double getSecondMotorAbsError() {
        return Math.abs(secondMotorProfile.calculate(getSecondMotorProfileTime()).position - getSecondMotorPosition());
    }

    private void setTargetPositionWithFeedforward(TalonFX motor, double position, double feedforward) {
        // TODO: check this
        final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(
                position, ArmConstants.USE_FOC,
                feedforward / motor.getSupplyVoltage().getValue(),
                0, false
        );
        motor.setControl(positionDutyCycle);
    }

    private double calculateFeedforward(ArmFeedforward feedforward, double position, double velocity) {
        return feedforward.calculate(Units.degreesToRadians(position), Units.degreesToRadians(velocity));
    }

    @Log(name = "First Motor Position")
    private double getFirstMotorPosition() {
        return Conversions.revolutionsToDegrees(firstMotor.getPosition().getValue());
    }

    @Log(name = "Second Motor Position")
    private double getSecondMotorPosition() {
        return Conversions.revolutionsToDegrees(secondMotor.getPosition().getValue());
    }

    private double getFirstMotorVelocity() {
        return Conversions.revolutionsToDegrees(firstMotor.getVelocity().getValue());
    }

    private double getSecondMotorVelocity() {
        return Conversions.revolutionsToDegrees(secondMotor.getVelocity().getValue());
    }

    private boolean isSecondJointRetracted() {
        return getSecondMotorPosition() >= ArmConstants.RETRACTED_DEGREES;
    }

    private double getFirstMotorSupplyCurrent() {
        return firstMotor.getSupplyCurrent().getValue();
    }

    private double getSecondMotorSupplyCurrent() {
        return secondMotor.getSupplyCurrent().getValue();
    }

    @Log(name = "First Motor Stator Current")
    private double getFirstMotorStatorCurrent() {
        return firstMotor.getStatorCurrent().getValue();
    }

    @Log(name = "Second Motor Stator Current")
    private double getSecondMotorStatorCurrent() {
        return secondMotor.getStatorCurrent().getValue();
    }

}

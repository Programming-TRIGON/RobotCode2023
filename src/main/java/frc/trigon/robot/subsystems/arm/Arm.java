package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.subsystems.LoggableSubsystemBase;
import frc.trigon.robot.utilities.Conversions;
import io.github.oblarg.oblog.annotations.Log;

public class Arm extends LoggableSubsystemBase {
    private static final Arm INSTANCE = new Arm();
    private final TalonFX
            firstJointMotor = ArmConstants.FIRST_JOINT_MASTER_MOTOR,
            secondJointMotor = ArmConstants.SECOND_JOINT_MOTOR;
    private TrapezoidProfile firstJointMotorProfile, secondJointMotorProfile;
    private double lastFirstJointProfileGenerationTimestamp, lastSecondJointProfileGenerationTimestamp;
    @Log
    private double secondJointSetpoint;
    @Log
    private boolean firstMotorAtGoal, secondMotorAtGoal, firstMotorVelocityAtGoal, secondMotorVelocityAtGoal;
    private double lastFirstJointSpeedFactor, lastSecondJointSpeedFactor;
    private String firstJointToMove = "";

    private Arm() {
        setCurrentLimits();
    }

    public static Arm getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        setTargetMotorPositions();
    }

    /**
     * @return true if both arm motors had reached their target position
     */
    public boolean atGoal() {
        firstMotorAtGoal = Math.abs(getFirstJointMotorError()) < ArmConstants.FIRST_JOINT_TOLERANCE;
        secondMotorAtGoal = Math.abs(getSecondJointMotorError()) < ArmConstants.SECOND_JOINT_TOLERANCE;
        firstMotorVelocityAtGoal = Math.abs(getFirstJointMotorVelocity()) < ArmConstants.FIRST_JOINT_VELOCITY_TOLERANCE;
        secondMotorVelocityAtGoal = Math.abs(getSecondJointMotorVelocity()) < ArmConstants.SECOND_JOINT_VELOCITY_TOLERANCE;

        return firstMotorAtGoal && secondMotorAtGoal && firstMotorVelocityAtGoal && secondMotorVelocityAtGoal;
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
    public CommandBase getGoToStateCommand(ArmConstants.ArmStates state, boolean byOrder, double firstMotorSpeedFactor, double secondMotorSpeedFactor) {
        return new StartEndCommand(
                () -> setTargetState(state, byOrder, firstMotorSpeedFactor, secondMotorSpeedFactor),
                () -> {},
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
                () -> {},
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

        firstJointMotor.getConfigurator().refresh(firstMotorConfig);
        firstMotorConfig.NeutralMode = mode;
        firstJointMotor.getConfigurator().apply(firstMotorConfig);

        secondJointMotor.getConfigurator().refresh(secondMotorConfig);
        secondMotorConfig.NeutralMode = mode;
        secondJointMotor.getConfigurator().apply(secondMotorConfig);
    }

    /**
     * Sets the neutral mode of the arm motors as their default neutral mode.
     */
    public void setNeutralMode() {
        final MotorOutputConfigs
            firstMotorConfig = new MotorOutputConfigs(),
            secondMotorConfig = new MotorOutputConfigs();

        firstJointMotor.getConfigurator().refresh(firstMotorConfig);
        firstMotorConfig.NeutralMode = ArmConstants.FIRST_JOINT_NEUTRAL_MODE;
        firstJointMotor.getConfigurator().apply(firstMotorConfig);

        secondJointMotor.getConfigurator().refresh(secondMotorConfig);
        secondMotorConfig.NeutralMode = ArmConstants.SECOND_JOINT_NEUTRAL_MODE;
        secondJointMotor.getConfigurator().apply(secondMotorConfig);
    }

    private void setTargetState(ArmConstants.ArmStates targetState) {
        setTargetState(targetState.firstMotorPosition, targetState.secondMotorPosition, true, 1, 1);
    }

    private void setTargetState(ArmConstants.ArmStates targetState, boolean byOrder, double firstMotorSpeedFactor, double secondMotorSpeedFactor) {
        setTargetState(targetState.firstMotorPosition, targetState.secondMotorPosition, byOrder, firstMotorSpeedFactor, secondMotorSpeedFactor);
    }

    private void setTargetState(double firstMotorPosition, double secondMotorPosition, boolean byOrder, double firstJointSpeedFactor, double secondJointSpeedFactor) {
        lastFirstJointSpeedFactor = firstJointSpeedFactor;
        lastSecondJointSpeedFactor = secondJointSpeedFactor;

        generateFirstMotorProfile(Rotation2d.fromDegrees(firstMotorPosition), firstJointSpeedFactor);
        generateSecondMotorProfile(Rotation2d.fromDegrees(secondMotorPosition), secondJointSpeedFactor);

        if (byOrder)
            firstJointToMove = getFirstJointMotorError() > 0 ? "first" : "second";
        else
            firstJointToMove = "";
    }

    private void setTargetMotorPositions() {
        if (this.getCurrentCommand() == null) {
            firstJointMotor.disable();
            secondJointMotor.disable();
        } else {
            setFirstJointPositionFromProfile();
            setSecondJointPositionFromProfile();
        }
    }

    private void generateFirstMotorProfile(Rotation2d targetAngle, double speedFactor) {
        firstJointMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.FIRST_JOINT_CONSTRAINTS, speedFactor),
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getFirstJointMotorAngle().getDegrees(), getFirstJointMotorVelocity())
        );

        lastFirstJointProfileGenerationTimestamp = Timer.getFPGATimestamp();
    }

    private void generateSecondMotorProfile(Rotation2d targetAngle, double speedFactor) {
        secondJointMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.SECOND_JOINT_CONSTRAINTS, speedFactor),
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getSecondJointMotorAngle().getDegrees(), getSecondJointMotorVelocity())
        );

        lastSecondJointProfileGenerationTimestamp = Timer.getFPGATimestamp();
    }

    private void setFirstJointPositionFromProfile() {
        if (firstJointMotorProfile == null) {
            firstJointMotor.stopMotor();
            return;
        }

        final double profileTime = getFirstMotorProfileTime();
        final TrapezoidProfile.State targetState = firstJointMotorProfile.calculate(profileTime);

        if (shouldStopFirstJointMotor(targetState)) {
            generateFirstMotorProfile(getFirstMotorProfileGoal(), lastFirstJointSpeedFactor);
            return;
        }

        final double feedforward = calculateFeedforward(ArmConstants.FIRST_JOINT_FEEDFORWARD, targetState.position, targetState.velocity);
        final double targetRevolutions = Conversions.degreesToRevolutions(targetState.position);

        setTargetMotorPositionWithFeedforward(firstJointMotor, targetRevolutions, feedforward);
    }

    private void setSecondJointPositionFromProfile() {
        if (secondJointMotorProfile == null) {
            secondJointMotor.stopMotor();
            return;
        }

        final double profileTime = getSecondMotorProfileTime();
        final TrapezoidProfile.State targetState = secondJointMotorProfile.calculate(profileTime);

        if (shouldStopSecondJointMotor(targetState)) {
            generateSecondMotorProfile(getSecondMotorProfileGoal(), lastSecondJointSpeedFactor);
            secondJointMotor.stopMotor();
            return;
        }

        final double feedforward = calculateFeedforward(
                ArmConstants.SECOND_JOINT_FEEDFORWARD,
                targetState.position + getSecondJointMotorAngle().getDegrees(),
                targetState.velocity
        );
        final double targetSystemRevolutions = Conversions.degreesToRevolutions(targetState.position);
        final double targetWheelRevolutions = Conversions.systemToMotor(targetSystemRevolutions, ArmConstants.SECOND_JOINT_GEAR_RATIO);

        setTargetMotorPositionWithFeedforward(secondJointMotor, targetWheelRevolutions, feedforward);
        secondJointSetpoint = targetState.position;
    }

    private boolean shouldStopFirstJointMotor(TrapezoidProfile.State targetState) {
        return isGoingToHitTheGround(targetState) ||
                (isNotFirstJointToMove("first") && isSecondJointJustStarting() && !isSecondJointRetracted());
    }

    private boolean shouldStopSecondJointMotor(TrapezoidProfile.State targetState) {
        return isGoingToHitTheGround(targetState) ||
                (isNotFirstJointToMove("second") && isFirstJointJustStarting() && isSecondJointRetracted());
    }

    private boolean isFirstJointJustStarting() {
        return getFirstJointProfileTimePercentage() < ArmConstants.RISE_PROFILE_COMPLETION_PERCENTAGE;
    }

    private boolean isSecondJointJustStarting() {
        return getSecondJointProfileTimePercentage() < ArmConstants.DESCEND_PROFILE_COMPLETION_PERCENTAGE;
    }

    private double getFirstJointProfileTimePercentage() {
        if (firstJointMotorProfile == null)
            return 1;
        return getFirstMotorProfileTime() / firstJointMotorProfile.totalTime();
    }

    private double getSecondJointProfileTimePercentage() {
        if (secondJointMotorProfile == null)
            return 1;
        return getSecondMotorProfileTime() / secondJointMotorProfile.totalTime();
    }

    private double getFirstMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastFirstJointProfileGenerationTimestamp;
    }

    private double getSecondMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastSecondJointProfileGenerationTimestamp;
    }

    private double getFirstJointMotorError() {
        return getFirstMotorProfileGoal().getDegrees() - getFirstJointMotorAngle().getDegrees();
    }

    private double getSecondJointMotorError() {
        return getSecondMotorProfileGoal().getDegrees() - getSecondJointMotorAngle().getDegrees();
    }

    private Rotation2d getFirstMotorProfileGoal() {
        if (firstJointMotorProfile == null)
            return getFirstJointMotorAngle();
        return Rotation2d.fromDegrees(firstJointMotorProfile.calculate(firstJointMotorProfile.totalTime()).position);
    }

    private Rotation2d getSecondMotorProfileGoal() {
        if (secondJointMotorProfile == null)
            return getSecondJointMotorAngle();
        return Rotation2d.fromDegrees(secondJointMotorProfile.calculate(secondJointMotorProfile.totalTime()).position);
    }

    private boolean isGoingToHitTheGround(TrapezoidProfile.State targetState) {
        return targetState.velocity < 0 && getCurrentEndEffectorLocation().getY() < ArmConstants.MINIMUM_END_EFFECTOR_HEIGHT;
    }

    private boolean isNotFirstJointToMove(String arm) {
        return !firstJointToMove.equals(arm) && !firstJointToMove.isBlank();
    }

    private Translation2d getCurrentEndEffectorLocation() {
        final Transform2d secondJointAngleTransform = new Transform2d(new Translation2d(), getSecondJointMotorAngle());
        final Pose2d secondJointLocation = getSecondJointLocationRelativeToGround().plus(secondJointAngleTransform);

        return secondJointLocation.plus(ArmConstants.SECOND_JOINT_TO_END_EFFECTOR).getTranslation();
    }

    private Pose2d getSecondJointLocationRelativeToGround() {
        final Pose2d firstJointPose = new Pose2d(new Translation2d(0, ArmConstants.FIRST_JOINT_HEIGHT), getFirstJointMotorAngle());
        return firstJointPose.plus(ArmConstants.FIRST_JOINT_TO_SECOND_JOINT);
    }

    private void setTargetMotorPositionWithFeedforward(TalonFX motor, double position, double feedforward) {
        // TODO: check this
        motor.setControl(new PositionVoltage(position, ArmConstants.USE_FOC, feedforward, 0, false));
    }

    private double calculateFeedforward(ArmFeedforward feedforward, double position, double velocity) {
        return feedforward.calculate(Units.degreesToRadians(position), Units.degreesToRadians(velocity));
    }

    @Log(name = "First Motor Position")
    private Rotation2d getFirstJointMotorAngle() {
        return Rotation2d.fromRotations(firstJointMotor.getPosition().getValue());
    }

    @Log(name = "Second Motor Position")
    private Rotation2d getSecondJointMotorAngle() {
        final double systemPosition = Conversions.motorToSystem(secondJointMotor.getPosition().getValue(), ArmConstants.SECOND_JOINT_GEAR_RATIO);

        return Rotation2d.fromRotations(systemPosition);
    }

    private double getFirstJointMotorVelocity() {
        return Conversions.revolutionsToDegrees(firstJointMotor.getVelocity().getValue());
    }

    private double getSecondJointMotorVelocity() {
        final double systemRevolutionsPerSecond = Conversions.motorToSystem(secondJointMotor.getVelocity().getValue(), ArmConstants.SECOND_JOINT_GEAR_RATIO);

        return Conversions.revolutionsToDegrees(systemRevolutionsPerSecond);
    }

    private boolean isSecondJointRetracted() {
        return getSecondJointMotorAngle().getDegrees() >= ArmConstants.RETRACTED_DEGREES;
    }

    private void setCurrentLimits() {
        ArmConstants.FIRST_JOINT_CURRENT_LIMIT_CONFIG.setup(
                () -> {
                    firstJointMotorProfile = null;
                    DriverStation.reportWarning("Arm first motor current draw is too high!\t" + firstJointMotor.getStatorCurrent(), false);
                }
        );
        ArmConstants.SECOND_JOINT_CURRENT_LIMIT_CONFIG.setup(
                () -> {
                    secondJointMotorProfile = null;
                    DriverStation.reportWarning("Arm second motor current draw is too high!\t" + secondJointMotor.getStatorCurrent(), false);
                }
        );
    }

}

package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmIO;
import frc.trigon.robot.subsystems.arm.talonfxarm.TalonFXArmIO;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static final Arm INSTANCE = new Arm();

    private final ArmIO armIO;
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();
    private TrapezoidProfile firstJointMotorProfile, secondJointMotorProfile;
    private double lastFirstJointProfileGenerationTimestamp, lastSecondJointProfileGenerationTimestamp;
    private double lastFirstJointSpeedFactor, lastSecondJointSpeedFactor;
    private String firstJointToMove = "";

    private Arm() {
        armIO = generateIO();
        setCurrentLimits();
    }

    public static Arm getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        setTargetMotorPositions();
        refreshInputs();
        updateNetworkTables();
    }

    /**
     * @return true if both arm motors had reached their target position
     */
    public boolean atGoal() {
        return Math.abs(getFirstJointMotorError()) < ArmConstants.FIRST_JOINT_TOLERANCE &&
                Math.abs(getSecondJointMotorError()) < ArmConstants.SECOND_JOINT_TOLERANCE &&
                Math.abs(getFirstJointMotorVelocity()) < ArmConstants.FIRST_JOINT_VELOCITY_TOLERANCE &&
                Math.abs(getSecondJointMotorVelocity()) < ArmConstants.SECOND_JOINT_VELOCITY_TOLERANCE;
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
        armIO.setNeutralMode(brake);
    }

    /**
     * Sets the neutral mode of the arm motors as their default neutral mode.
     */
    public void setNeutralMode() {
        armIO.setNeutralMode(ArmConstants.DEFAULT_NEUTRAL_MODE);
    }

    private void refreshInputs() {
        armIO.updateInputs(armInputs);
        Logger.getInstance().processInputs(getLoggingPath(), armInputs);
    }

    private void updateNetworkTables() {
        Logger.getInstance().recordOutput(getLoggingPath() + "firstMotorAtGoal", Math.abs(getFirstJointMotorError()) < ArmConstants.FIRST_JOINT_TOLERANCE);
        Logger.getInstance().recordOutput(getLoggingPath() + "secondMotorAtGoal", Math.abs(getSecondJointMotorError()) < ArmConstants.SECOND_JOINT_TOLERANCE);
        Logger.getInstance().recordOutput(getLoggingPath() + "firstMotorVelocityAtGoal", Math.abs(getFirstJointMotorVelocity()) < ArmConstants.FIRST_JOINT_VELOCITY_TOLERANCE);
        Logger.getInstance().recordOutput(getLoggingPath() + "secondMotorVelocityAtGoal", Math.abs(getSecondJointMotorVelocity()) < ArmConstants.SECOND_JOINT_VELOCITY_TOLERANCE);

        Logger.getInstance().recordOutput(getLoggingPath() + "firstJointPose", getFirstJointComponentPose());
        Logger.getInstance().recordOutput(getLoggingPath() + "secondJointPose", getSecondJointComponentPose());

        updateMechanism();
    }

    private void updateMechanism() {
        ArmConstants.FIRST_JOINT_LIGAMENT.setAngle(getFirstJointMotorAngle().getDegrees());
        ArmConstants.SECOND_JOINT_LIGAMENT.setAngle(getSecondJointMotorAngle().getDegrees());

        Logger.getInstance().recordOutput("ArmMechanism", ArmConstants.ARM_MECHANISM);
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
            armIO.stop();
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
            armIO.stopFirstJoint();
            return;
        }

        final double profileTime = getFirstMotorProfileTime();
        final TrapezoidProfile.State targetState = firstJointMotorProfile.calculate(profileTime);

        if (shouldStopFirstJointMotor(targetState)) {
            generateFirstMotorProfile(getFirstMotorProfileGoal(), lastFirstJointSpeedFactor);
            return;
        }

        armIO.setTargetFirstJointPosition(targetState.position, targetState.velocity);
        Logger.getInstance().recordOutput("firstJointSetpoint", targetState.position);
    }

    private void setSecondJointPositionFromProfile() {
        if (secondJointMotorProfile == null) {
            armIO.stopSecondJoint();
            return;
        }

        final double profileTime = getSecondMotorProfileTime();
        final TrapezoidProfile.State targetState = secondJointMotorProfile.calculate(profileTime);

        if (shouldStopSecondJointMotor(targetState)) {
            generateSecondMotorProfile(getSecondMotorProfileGoal(), lastSecondJointSpeedFactor);
            armIO.stopSecondJoint();
            return;
        }

        armIO.setTargetSecondJointPosition(targetState.position, targetState.velocity);
        Logger.getInstance().recordOutput("secondJointSetpoint", targetState.position);
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
        return Logger.getInstance().getRealTimestamp() - lastFirstJointProfileGenerationTimestamp;
    }

    private double getSecondMotorProfileTime() {
        return Logger.getInstance().getRealTimestamp() - lastSecondJointProfileGenerationTimestamp;
    }

    private double getFirstJointMotorError() {
        return getFirstMotorProfileGoal().getDegrees() - getFirstJointMotorAngle().getDegrees();
    }

    private double getSecondJointMotorError() {
        return getSecondJointMotorAngle().getDegrees() - getSecondMotorProfileGoal().getDegrees();
    }

    private Pose3d getFirstJointComponentPose() {
        return new Pose3d(
                new Translation3d(0, 0, ArmConstants.FIRST_JOINT_HEIGHT),
                new Rotation3d(0, getFirstJointMotorAngle().getRadians(), 0)
        );
    }

    private Pose3d getSecondJointComponentPose() {
        return new Pose3d(
                new Translation3d(getSecondJointLocationRelativeToGround().getX(), 0, getSecondJointLocationRelativeToGround().getY()),
                new Rotation3d(0, 0, getSecondJointLocationRelativeToGround().getRotation().getRadians())
        );
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

    private boolean isNotFirstJointToMove(String joint) {
        return !firstJointToMove.equals(joint) && !firstJointToMove.isBlank();
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

    private Rotation2d getFirstJointMotorAngle() {
        return Rotation2d.fromDegrees(armInputs.firstJointPositionDegrees);
    }

    private Rotation2d getSecondJointMotorAngle() {
        return Rotation2d.fromDegrees(armInputs.secondJointPositionDegrees);
    }

    private double getFirstJointMotorVelocity() {
        return armInputs.firstJointVelocityDegreesPerSecond;
    }

    private double getSecondJointMotorVelocity() {
        return armInputs.secondJointVelocityDegreesPerSecond;
    }

    private boolean isSecondJointRetracted() {
        return getSecondJointMotorAngle().getDegrees() >= ArmConstants.RETRACTED_DEGREES;
    }

    private void setCurrentLimits() {
        armIO.setupLimits(
                () -> {
                    firstJointMotorProfile = null;
                    DriverStation.reportWarning("Arm first motor current draw is too high!\t" + armInputs.firstJointStatorCurrent, false);
                },
                () -> {
                    secondJointMotorProfile = null;
                    DriverStation.reportWarning("Arm second motor current draw is too high!\t" + armInputs.secondJointStatorCurrent, false);
                }
        );
    }
    
    private String getLoggingPath() {
        return "Arm/";
    }

    private ArmIO generateIO() {
        if (ConfigurationConstants.IS_REPLAY)
            return new ArmIO();

        if (ConfigurationConstants.ROBOT_TYPE == ConfigurationConstants.RobotType.TRIHARD) {
            return new TalonFXArmIO();
        }

        return new SimulationArmIO();
    }
}

package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.subsystems.powerdistribution.PowerDistributionManager;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.LargeFreedomBangBangController;

import java.util.logging.Logger;

public class Arm extends SubsystemBase {
    private static final Arm INSTANCE = new Arm();
    private final WPI_TalonFX
            firstMotor = ArmConstants.FIRST_JOINT_FIRST_MOTOR,
            secondMotor = ArmConstants.SECOND_JOINT_MOTOR;
    private final ArmFeedforward
            firstMotorFeedforward = ArmConstants.FIRST_JOINT_FEEDFORWARD,
            secondMotorFeedforward = ArmConstants.SECOND_JOINT_FEEDFORWARD;
    private final LargeFreedomBangBangController secondMotorBBController = ArmConstants.SECOND_JOINT_BANG_BANG_CONTROLLER;
    public TrapezoidProfile firstMotorProfile, secondMotorProfile;
    private double firstMotorProfileLastSetTime, secondMotorProfileLastSetTime;
    private String firstArmToMove = "";

    private Arm() {
        PowerDistributionManager.getInstance().setPortRequirements(
                15,
                0.2,
                10,
                () -> {
                    firstMotorProfile = null;
                    Logger.getGlobal().warning("Arm first motor current draw is too high!");
                }
        );
        PowerDistributionManager.getInstance().setPortRequirements(
                1,
                0.2,
                10,
                () -> {
                    secondMotorProfile = null;
                    DriverStation.reportWarning("Arm second motor current draw is too high!", false);
                }
        );

        SmartDashboard.putData("second motor bb controller", secondMotorBBController);
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
     * @param state the target state
     * @return the command
     */
    public Command getGoToStateCommand(ArmStates state, boolean byOrder) {
        return new StartEndCommand(
                () -> setTargetState(state, byOrder),
                () -> {},
                this
        );
    }

    public Command getGoToStateCommand(ArmStates state) {
        return new StartEndCommand(
                () -> setTargetState(state),
                () -> {},
                this
        );
    }

    private void setTargetState(ArmStates targetState, boolean byOrder) {
        setTargetState(targetState.firstMotorPosition, targetState.secondMotorPosition, byOrder
        );
    }

    private void setTargetState(ArmStates targetState) {
        setTargetState(targetState.firstMotorPosition, targetState.secondMotorPosition, false);
    }

    /**
     * Constructs a command that sets the target position to the arm.
     *
     * @param firstJointAngle  the angle of the first joint
     * @param secondJointAngle the angle of the second joint
     * @return the command
     */
    public Command getGoToPositionCommand(double firstJointAngle, double secondJointAngle, boolean byOrder) {
        return new StartEndCommand(
                () -> setTargetState(firstJointAngle, secondJointAngle, byOrder),
                () -> {},
                this
        );
    }

    public Command getGoToPositionCommand(double firstJointAngle, double secondJointAngle) {
        return new StartEndCommand(
                () -> setTargetState(firstJointAngle, secondJointAngle, false),
                () -> {},
                this
        );
    }

    public void setNeutralMode(boolean brake) {
        NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
        firstMotor.setNeutralMode(mode);
        secondMotor.setNeutralMode(mode);
    }

    public void setNeutralMode() {
        firstMotor.setNeutralMode(ArmConstants.FIRST_JOINT_NEUTRAL_MODE);
        secondMotor.setNeutralMode(ArmConstants.SECOND_JOINT_NEUTRAL_MODE);
    }

    private void setTargetState(double firstMotorPosition, double secondMotorPosition, boolean byOrder) {
        generateFirstMotorProfile(firstMotorPosition);
        generateSecondMotorProfile(secondMotorPosition);
        if(byOrder)
            firstArmToMove = getFirstMotorDistanceToGoal() > 0 ? "first" : "second";
        else
            firstArmToMove = "";
        System.out.println(firstArmToMove + "____________________________________");
    }

    private void generateFirstMotorProfile(double position) {
        firstMotorProfile = new TrapezoidProfile(
                ArmConstants.FIRST_JOINT_CONSTRAINTS,
                new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(getFirstMotorPosition(), getFirstMotorVelocity())
        );
        firstMotorProfileLastSetTime = Timer.getFPGATimestamp();
    }

    private void generateSecondMotorProfile(double position) {
        secondMotorProfile = new TrapezoidProfile(
                ArmConstants.SECOND_JOINT_CONSTRAINTS,
                new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(getSecondMotorPosition(), getSecondMotorVelocity())
        );
        secondMotorBBController.setSetpoint(position);
        secondMotorProfileLastSetTime = Timer.getFPGATimestamp();
    }

    private void setTargetMotorPositions() {
        if(this.getCurrentCommand() == null) {
            firstMotor.disable();
            secondMotor.disable();
        } else {
            setFirstMotorPositionFromProfile();
            setSecondMotorPositionFromProfile();
        }
    }

    private void setFirstMotorPositionFromProfile() {
        if(firstMotorProfile == null)
            return;

        double profileTime = Timer.getFPGATimestamp() - firstMotorProfileLastSetTime;

        TrapezoidProfile.State targetState = firstMotorProfile.calculate(profileTime);

        var endEffector = getEndEffectorLocation(targetState.position, getSecondMotorPosition());
        boolean firstCon = targetState.velocity < 0 && (endEffector.getY() < 20 ||
                (endEffector.getX() < 40 && endEffector.getY() < 30));
        boolean secondCon = !firstArmToMove.equals("first") && !firstArmToMove.isBlank() && (Math.abs(getSecondMotorDistanceToGoal()) > 10 && getSecondMotorPosition() < 140);
        if(firstCon || secondCon) {
            generateFirstMotorProfile(getFirstMotorGoal());
            System.out.println("regenerating for first, target velocity:" + targetState.velocity + ". " + (firstCon ? "firstCon" : ("SecondCon " + getFirstMotorDistanceToGoal())));
            return;
        }

        double feedforward = calculateFeedforward(firstMotorFeedforward, targetState.position, targetState.velocity);
        double targetPosition = Conversions.degreesToMagTicks(targetState.position);

        setTargetPositionWithFeedforwardForTalonFx(firstMotor, targetPosition, feedforward);
    }

    private void setSecondMotorPositionFromProfile() {
        if(secondMotorProfile == null)
            return;

        double profileTime = Timer.getFPGATimestamp() - secondMotorProfileLastSetTime;
        TrapezoidProfile.State targetState = secondMotorProfile.calculate(profileTime);

        var endEffector = getEndEffectorLocation(getFirstMotorPosition(), targetState.position);

        boolean firstCon = targetState.velocity < 0 && (endEffector.getY() < 20 ||
                (endEffector.getX() < 40 && endEffector.getY() < 30));
        boolean secondCon = !firstArmToMove.equals("second") && !firstArmToMove.isBlank() && (Math.abs(getFirstMotorDistanceToGoal()) > 10 && getFirstMotorPosition() > -60);
        if(firstCon || secondCon) {
            generateSecondMotorProfile(getSecondMotorGoal());
            System.out.println("regenerating for second, target velocity:" + targetState.velocity + ". " + (firstCon ? "firstCon" : ("SecondCon " + getFirstMotorDistanceToGoal() )));
            return;
        }

        double targetPosition = Conversions.degreesToMagTicks(targetState.position);
        double feedforward = calculateFeedforward(
                secondMotorFeedforward,
                targetState.position + getFirstMotorPosition(),
                targetState.velocity
        );

        secondMotor.configPeakOutputForward(0.1);
        setTargetPositionWithFeedforwardForTalonFx(secondMotor, targetPosition, feedforward);
        SmartDashboard.putNumber("second motor setpoint", targetState.position);
    }

    private double getFirstMotorDistanceToGoal() {
        return getFirstMotorGoal() - getFirstMotorPosition();
    }

    private Translation2d getEndEffectorLocation(double alpha, double beta) {
        var secondJointLocation =
                new Pose2d(new Translation2d(0, ArmConstants.FIRST_JOINT_HEIGHT), Rotation2d.fromDegrees(alpha)).plus(new Transform2d(new Translation2d(ArmConstants.FIRST_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0)));
        return secondJointLocation.plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(beta))).plus(new Transform2d(new Translation2d(ArmConstants.SECOND_JOINT_LENGTH, 0), Rotation2d.fromDegrees(0))).getTranslation();
    }

    private double getSecondMotorDistanceToGoal() {
        return getSecondMotorGoal() - getSecondMotorPosition();
    }

    private double getSecondMotorGoal() {
        if(secondMotorProfile == null)
            return getSecondMotorPosition();
        return secondMotorProfile.calculate(secondMotorProfile.totalTime()).position;
    }

    private double getFirstMotorGoal() {
        if(firstMotorProfile == null)
            return getFirstMotorPosition();
        return firstMotorProfile.calculate(firstMotorProfile.totalTime()).position;
    }

    private double getSecondMotorAbsError() {
        return Math.abs(secondMotorProfile.calculate(Timer.getFPGATimestamp() - secondMotorProfileLastSetTime).position - getSecondMotorPosition());
    }

    private double calculateBangBangPower() {
        //return secondMotorBBController.calculate(getSecondMotorPosition()) * ArmConstants.BANG_BANG_POWER;
        return Math.min(Double.POSITIVE_INFINITY, 0.4 * Math.signum((getSecondMotorGoal() - getSecondMotorPosition())));
    }

    private void setTargetPositionWithFeedforwardForTalonFx(WPI_TalonFX motor, double position, double feedforward) {
        motor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedforward / motor.getBusVoltage());
    }

    private double calculateFeedforward(ArmFeedforward feedforward, double position, double velocity) {
        return feedforward.calculate(Units.degreesToRadians(position), Units.degreesToRadians(velocity));
    }

    private double getFirstMotorPosition() {
        return Conversions.magTicksToDegrees(firstMotor.getSelectedSensorPosition());
    }

    private double getSecondMotorPosition() {
        return Conversions.magTicksToDegrees(secondMotor.getSelectedSensorPosition());
    }

    private double getFirstMotorVelocity() {
        return Conversions.magTicksToDegrees(Conversions.perHundredMsToPerSecond(firstMotor.getSelectedSensorVelocity()));
    }

    private double getSecondMotorVelocity() {
        return Conversions.magTicksToDegrees(Conversions.perHundredMsToPerSecond(secondMotor.getSelectedSensorVelocity()));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("First Motor Position", this::getFirstMotorPosition, this::generateFirstMotorProfile);
        builder.addDoubleProperty("First Motor Velocity", this::getFirstMotorVelocity, null);
        builder.addDoubleProperty("Second Motor Position", this::getSecondMotorPosition, this::generateSecondMotorProfile);
        builder.addDoubleProperty("Second Motor Velocity", this::getSecondMotorVelocity, null);
        builder.addDoubleProperty("End Effector X", () -> getEndEffectorLocation(getFirstMotorPosition(), getSecondMotorPosition()).getX(), null);
        builder.addDoubleProperty("End Effector Y", () -> getEndEffectorLocation(getFirstMotorPosition(), getSecondMotorPosition()).getY(), null);

    }

    public enum ArmStates {
        CLOSED(-79.453125, 156.093750),
        CLOSED_COLLECTING(-79.453125, 86.308594),
        THIRD_STATE(90, 225);
        //TODO: config real states
        public final double firstMotorPosition, secondMotorPosition;

        ArmStates(double firstMotorPosition, double secondMotorPosition) {
            this.firstMotorPosition = firstMotorPosition;
            this.secondMotorPosition = secondMotorPosition;
        }
    }
}

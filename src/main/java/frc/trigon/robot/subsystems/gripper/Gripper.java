package frc.trigon.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.subsystems.gripper.simulationgripper.SimulationGripperIO;
import frc.trigon.robot.subsystems.gripper.talonfxgripper.TalonFXGripperIO;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static final Gripper INSTANCE = new Gripper();

    private final GripperIO gripperIO;
    private final GripperInputsAutoLogged gripperInputsAutoLogged = new GripperInputsAutoLogged();

    public static Gripper getInstance() {
        return INSTANCE;
    }

    private GripperConstants.GripperState state = GripperConstants.GripperState.STOP;

    private Gripper() {
        gripperIO = generateIO();
        setPowerDistributionPortRequirements();
        setDefaultCommand(
                new StartEndCommand(
                        () -> setState(GripperConstants.GripperState.HOLD),
                        () -> setState(GripperConstants.GripperState.STOP),
                        this
                )
        );
    }

    @Override
    public void periodic() {
        gripperIO.updateInputs(gripperInputsAutoLogged);
        Logger.getInstance().processInputs(getLoggingPath(), gripperInputsAutoLogged);
    }

    /**
     * @return a command that makes the gripper collect, and stops the gripper at the end of it
     */
    public CommandBase getCollectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.COLLECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        ));
    }

    /**
     * @return a command that makes the gripper collect slowly, and stops the gripper at the end of it
     */
    public CommandBase getSlowCollectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.SLOW_COLLECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        ));
    }

    /**
     * @return a command that makes the gripper eject, and stops the gripper at the end of it
     */
    public CommandBase getEjectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        ));
    }

    /**
     * @return a command that makes the gripper eject slowly, and stops the gripper at the end of it
     */
    public CommandBase getSlowEjectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.SLOW_EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        ));
    }

    /**
     * @return a command that makes the gripper hold, and stops the gripper at the end of it
     */
    public CommandBase getHoldCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.HOLD),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        ));
    }

    /**
     * @return a command that stops the gripper
     */
    public InstantCommand getStopCommand() {
        return new InstantCommand(
                () -> setState(GripperConstants.GripperState.STOP),
                this
        );
    }

    public CommandBase getFullEjectCommand(){
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.FULL_EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        ));
    }

    public boolean isHolding() {
        return state == GripperConstants.GripperState.HOLD;
    }

    private void setPowerDistributionPortRequirements() {
        gripperIO.configHoldTrigger(
                () -> {
                    if(state.power < 0)
                        setState(GripperConstants.GripperState.HOLD);
                }
        );
    }

    private void setState(GripperConstants.GripperState state) {
        this.state = state;
        gripperIO.setTargetPower(state.power);
    }

    private String getLoggingPath() {
        return "Gripper";
    }

    private GripperIO generateIO() {
        if (ConfigurationConstants.IS_REPLAY)
            return new GripperIO() {};

        if (ConfigurationConstants.ROBOT_TYPE == ConfigurationConstants.RobotType.TRIHARD)
            return new TalonFXGripperIO();

        return new SimulationGripperIO();
    }
}

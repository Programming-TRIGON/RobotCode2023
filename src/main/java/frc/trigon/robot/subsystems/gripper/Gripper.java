package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.subsystems.LoggableSubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Gripper extends LoggableSubsystemBase {
    private static final Gripper INSTANCE = new Gripper();
    private final TalonFX motor = GripperConstants.MOTOR;

    public static Gripper getInstance() {
        return INSTANCE;
    }

    private GripperConstants.GripperState state = GripperConstants.GripperState.STOP;

    private Gripper() {
        setPowerDistributionPortRequirements();
        setDefaultCommand(
                new StartEndCommand(
                        () -> setState(GripperConstants.GripperState.HOLD),
                        () -> setState(GripperConstants.GripperState.STOP),
                        this
                )
        );
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
            GripperConstants.HOLD_TRIGGER_CONFIG.setup(
                    () -> {
                        if(state.power < 0)
                            setState(GripperConstants.GripperState.HOLD);
                    }
            );
    }

    @Log
    private double getStatorCurrent() {
        return motor.getStatorCurrent().getValue();
    }

    private void setState(GripperConstants.GripperState state) {
        this.state = state;
        motor.set(state.power);
    }
}

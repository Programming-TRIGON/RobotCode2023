package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.subsystems.LoggableSubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Gripper extends LoggableSubsystemBase {
    private static final Gripper INSTANCE = new Gripper();
    private final WPI_TalonFX motor = GripperConstants.MOTOR;

    public static Gripper getInstance() {
        return INSTANCE;
    }

    private GripperConstants.GripperState state = GripperConstants.GripperState.STOP;

    private Gripper() {
        setPowerDistributionPortRequirements();
    }

    /**
     * @return a command that makes the gripper collect, and stops the gripper at the end of it
     */
    public StartEndCommand getCollectCommand() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.COLLECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        );
    }

    /**
     * @return a command that makes the gripper collect slowly, and stops the gripper at the end of it
     */
    public StartEndCommand getSlowCollectCommand() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.SLOW_COLLECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        );
    }



    /**
     * @return a command that makes the gripper eject, and stops the gripper at the end of it
     */
    public StartEndCommand getEjectCommand() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        );
    }

    /**
     * @return a command that makes the gripper eject slowly, and stops the gripper at the end of it
     */
    public StartEndCommand getSlowEjectCommand() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.SLOW_EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        );
    }

    /**
     * @return a command that makes the gripper hold, and stops the gripper at the end of it
     */
    public StartEndCommand getHoldCommand() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.HOLD),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        );
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
        return motor.getStatorCurrent();
    }

    private void setState(GripperConstants.GripperState state) {
        this.state = state;
        motor.set(state.power);
    }
}

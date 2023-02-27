package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.subsystems.powerdistribution.PowerDistributionManager;

public class Gripper extends SubsystemBase {
    private static final Gripper INSTANCE = new Gripper();
    private final WPI_TalonFX motor = GripperConstants.MOTOR;

    public static Gripper getInstance() {
        return INSTANCE;
    }

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
     * @return a command that makes the gripper hold, and stops the gripper at the end of it
     */
    public StartEndCommand getHoldCommand() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.HOLD),
                () -> setState(GripperConstants.GripperState.STOP),
                this
        );
    }

    private void setPowerDistributionPortRequirements() {
        PowerDistributionManager.getInstance().setPortRequirements(
                GripperConstants.HOLD_TRIGGER_CONFIG,
                () -> setState(GripperConstants.GripperState.HOLD)
        );
    }

    private void setState(GripperConstants.GripperState state) {
        motor.set(state.power);
    }
}

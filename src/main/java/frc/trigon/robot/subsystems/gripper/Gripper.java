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
        PowerDistributionManager.getInstance().setPortRequirements(
                GripperConstants.POWER_DISTRIBUTION_CONFIG,
                () -> setState(GripperConstants.GripperState.HOLD)
        );
    }

    /**
     * @return a command that runs the gripper at the collect power
     */
    public StartEndCommand collect() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.COLLECT),
                () -> setState(GripperConstants.GripperState.STOPPED)
        );
    }

    /**
     * @return a command that runs the gripper at the eject power
     */
    public StartEndCommand eject() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.EJECT),
                () -> setState(GripperConstants.GripperState.STOPPED)
        );
    }

    /**
     * @return a command that runs the gripper at the hold power
     */
    public StartEndCommand hold() {
        return new StartEndCommand(
                () -> setState(GripperConstants.GripperState.HOLD),
                () -> setState(GripperConstants.GripperState.STOPPED)
        );
    }

    private void setState(GripperConstants.GripperState state) {
        motor.set(state.power);
    }
}

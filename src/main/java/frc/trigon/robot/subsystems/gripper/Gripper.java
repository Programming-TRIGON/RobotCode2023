package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.PowerDistributionManager;

public class Gripper extends SubsystemBase {
    private final WPI_TalonFX motor = GripperConstants.MOTOR;

    private static final Gripper INSTANCE = new Gripper();

    public static Gripper getInstance() {
        return INSTANCE;
    }

    private Gripper() {
        PowerDistributionManager.getInstance().setPortRequirements(
                GripperConstants.CURRENT_LIMIT_CONFIG,
                () -> setPower(GripperConstants.HOLD_POWER)
        );
    }

    private void setPower(double power) {
        motor.set(power);
    }

    /**
     * @return a command that runs the gripper at the collect power
     */
    public StartEndCommand collect() {
        return new StartEndCommand(
                () -> setPower(GripperConstants.COLLECT_POWER),
                () -> setPower(0)
        );
    }

    /**
     * @return a command that runs the gripper at the eject power
     */
    public StartEndCommand eject() {
        return new StartEndCommand(
                () -> setPower(GripperConstants.EJECT_POWER),
                () -> setPower(0)
        );
    }

    /**
     * @return a command that runs the gripper at the hold power
     */
    public StartEndCommand hold() {
        return new StartEndCommand(
                () -> setPower(GripperConstants.HOLD_POWER),
                () -> setPower(0)
        );
    }
}

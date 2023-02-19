package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.trigon.robot.utilities.PowerDistributionManager;

public class GripperConstants {
    private static final int MOTOR_PORT = 13;
    private static final int PD_PORT = 0; //TODO

    static final WPI_TalonFX MOTOR = new WPI_TalonFX(MOTOR_PORT);

    static final double
            COLLECT_POWER = -0.9,
            EJECT_POWER = 0.9,
            HOLD_POWER = -0.075;

    static final PowerDistributionManager.CurrentLimitConfig CURRENT_LIMIT_CONFIG =
            new PowerDistributionManager.CurrentLimitConfig(PD_PORT, 0.2, 4);
}

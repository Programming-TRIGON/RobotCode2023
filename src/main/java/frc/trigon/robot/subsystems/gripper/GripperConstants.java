package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.trigon.robot.subsystems.powerdistribution.PowerDistributionManager;

public class GripperConstants {
    private static final int MOTOR_ID = 13;
    private static final int POWER_DISTRIBUTION_PORT = 1;
    private static final double
            HOLD_TRIGGER_DURATION = 0.2,
            HOLD_TRIGGER_CURRENT = 4;
    static final PowerDistributionManager.CurrentLimitConfig CURRENT_LIMIT_CONFIG = new PowerDistributionManager.CurrentLimitConfig(
            POWER_DISTRIBUTION_PORT,
            HOLD_TRIGGER_DURATION,
            HOLD_TRIGGER_CURRENT
    );

    static final WPI_TalonFX MOTOR = new WPI_TalonFX(MOTOR_ID);

    private static final double
            COLLECT_POWER = -0.9,
            EJECT_POWER = 0.9,
            HOLD_POWER = -0.075;

    enum GripperState {
        STOPPED(0),
        COLLECT(COLLECT_POWER),
        EJECT(EJECT_POWER),
        HOLD(HOLD_POWER);

        final double power;

        GripperState(double power) {
            this.power = power;
        }
    }
}

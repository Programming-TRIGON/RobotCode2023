package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.trigon.robot.subsystems.powerdistribution.PowerDistributionManager;

public class GripperConstants {
    private static final int MOTOR_ID = 13;
    private static final int POWER_DISTRIBUTION_PORT = 2;
    private static final double
            HOLD_TRIGGER_DURATION = 0.2,
            HOLD_TRIGGER_CURRENT = 6;
    static final PowerDistributionManager.CurrentLimitConfig POWER_DISTRIBUTION_CONFIG = new PowerDistributionManager.CurrentLimitConfig(
            POWER_DISTRIBUTION_PORT,
            HOLD_TRIGGER_DURATION,
            HOLD_TRIGGER_CURRENT
    );

    static final WPI_TalonFX MOTOR = new WPI_TalonFX(MOTOR_ID);

    enum GripperState {
        STOPPED(0),
        COLLECT(-0.9),
        EJECT(0.9),
        HOLD(-0.075);

        final double power;

        GripperState(double power) {
            this.power = power;
        }
    }
}

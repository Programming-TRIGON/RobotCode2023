package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class GripperConstants {
    private static final int MOTOR_ID = 13;
    static final int POWER_DISTRIBUTION_PORT = 1;
    static final double
            POWER_DISTRIBUTION_TRIGGER_DURATION = 0.2,
            POWER_DISTRIBUTION_TRIGGER_CURRENT = 4;

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

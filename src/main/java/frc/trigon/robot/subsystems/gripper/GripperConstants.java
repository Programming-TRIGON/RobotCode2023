package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.trigon.robot.utilities.CurrentWatcher;

public class GripperConstants {
    private static final int MOTOR_ID = 13;
    private static final int POWER_DISTRIBUTION_PORT = 2;

    private static final boolean INVERTED = false;

    private static final double
            HOLD_TRIGGER_DURATION = 0.1,
            HOLD_TRIGGER_CURRENT = 6;

    static final WPI_TalonFX MOTOR = new WPI_TalonFX(MOTOR_ID);

    static final CurrentWatcher.CurrentWatcherConfig HOLD_TRIGGER_CONFIG = new CurrentWatcher.CurrentWatcherConfig(
            MOTOR::getStatorCurrent,
            HOLD_TRIGGER_DURATION,
            HOLD_TRIGGER_CURRENT
    );

    static {
        MOTOR.configFactoryDefault();

        MOTOR.setInverted(INVERTED);
        MOTOR.setNeutralMode(NeutralMode.Brake);
    }

    enum GripperState {
        STOP(0),
        COLLECT(-0.9),
        EJECT(0.43),
        HOLD(-0.1);

        final double power;

        GripperState(double power) {
            this.power = power;
        }
    }
}

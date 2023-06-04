package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.utilities.CurrentWatcher;

public class GripperConstants {
    private static final int MOTOR_ID = 13;
    private static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    private static final double
            HOLD_TRIGGER_DURATION = 0.05,
            HOLD_TRIGGER_CURRENT = 40,
            CURRENT_LIMIT = 32;

    static final TalonFX MOTOR = new TalonFX(MOTOR_ID);

    static final CurrentWatcher.CurrentWatcherConfig HOLD_TRIGGER_CONFIG = new CurrentWatcher.CurrentWatcherConfig(
            () -> MOTOR.getStatorCurrent().getValue(),
            HOLD_TRIGGER_CURRENT,
            HOLD_TRIGGER_DURATION
    );

    static {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = INVERTED;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    enum GripperState {
        STOP(0),
        COLLECT(-0.9),
        SLOW_COLLECT(-0.3),
        EJECT(0.43),
        FULL_EJECT(1),
        SLOW_EJECT(0.12),
        HOLD(-0.1);

        final double power;

        GripperState(double power) {
            this.power = power;
        }
    }
}

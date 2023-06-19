package frc.trigon.robot.subsystems.gripper.talonfxgripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.utilities.CurrentWatcher;

public class TalonFXGripperConstants {
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
}

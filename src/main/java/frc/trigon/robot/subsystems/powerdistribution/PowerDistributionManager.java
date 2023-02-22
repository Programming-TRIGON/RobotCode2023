package frc.trigon.robot.subsystems.powerdistribution;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

/**
 * A subsystem that manages the power distribution.
 * This subsystem will run the callback when a trigger is reached.
 */
public class PowerDistributionManager extends SubsystemBase {
    private final static PowerDistributionManager INSTANCE = new PowerDistributionManager();

    private final PowerDistribution powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    private final HashMap<CurrentLimitConfig, Runnable> requirements = new HashMap<>();

    private PowerDistributionManager() {
    }

    public static PowerDistributionManager getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        requirements.keySet().forEach(this::checkCurrent);
    }

    /**
     * Sets the port requirements for the specified port.
     *
     * @param powerDistributionPort   the power distribution port
     * @param callbackTriggerDuration the callback trigger duration
     * @param callbackTriggerCurrent  the callback trigger current
     * @param callback                a callback to run when a trigger is reached
     */
    public void setPortRequirements(int powerDistributionPort, double callbackTriggerDuration, double callbackTriggerCurrent, Runnable callback) {
        setPortRequirements(new CurrentLimitConfig(powerDistributionPort, callbackTriggerDuration, callbackTriggerCurrent), callback);
    }

    /**
     * Sets the port requirements for the specified current limit's port.
     *
     * @param config   the current limit config
     * @param callback a callback to run when a trigger is reached
     */
    public void setPortRequirements(CurrentLimitConfig config, Runnable callback) {
        requirements.put(config, callback);
    }

    private void checkCurrent(CurrentLimitConfig config) {
        double current = powerDistribution.getCurrent(config.powerDistributionPort);
        double timeDifference = Timer.getFPGATimestamp() - config.lastUpdatedTimestamp;

        if (current < config.callbackTriggerCurrent) {
            config.lastUpdatedTimestamp = Timer.getFPGATimestamp();
            return;
        }

        if (timeDifference >= config.callbackTriggerDuration) {
            Runnable runnable = requirements.get(config);
            if (runnable != null)
                runnable.run();
        }
    }

    public static class CurrentLimitConfig {
        final int powerDistributionPort;
        final double callbackTriggerDuration, callbackTriggerCurrent;
        double lastUpdatedTimestamp;

        public CurrentLimitConfig(int powerDistributionPort, double callbackTriggerDuration, double callbackTriggerCurrent) {
            this.powerDistributionPort = powerDistributionPort;
            this.callbackTriggerDuration = callbackTriggerDuration;
            this.callbackTriggerCurrent = callbackTriggerCurrent;
        }
    }
}

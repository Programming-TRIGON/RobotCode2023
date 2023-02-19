package frc.trigon.robot.subsystems.powerdistribution;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

public class PowerDistributionManager extends SubsystemBase {
    private final static PowerDistributionManager INSTANCE = new PowerDistributionManager();

    private final PowerDistribution powerDistribution = new PowerDistribution();
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
     * @param powerDistributionPort the power distribution port
     * @param triggerDuration       the trigger duration
     * @param triggerCurrent        the trigger current
     * @param callback              a callback to run when the current limit is reached
     */
    public void setPortRequirements(int powerDistributionPort, double triggerDuration, double triggerCurrent, Runnable callback) {
        setPortRequirements(new CurrentLimitConfig(powerDistributionPort, triggerDuration, triggerCurrent), callback);
    }

    /**
     * Sets the port requirements for the specified current limit's port.
     *
     * @param config   the current limit config
     * @param callback a callback to run when the current limit is reached
     */
    public void setPortRequirements(CurrentLimitConfig config, Runnable callback) {
        requirements.put(config, callback);
    }

    private void checkCurrent(CurrentLimitConfig config) {
        double current = powerDistribution.getCurrent(config.powerDistribution);
        double triggeredDuration = Timer.getFPGATimestamp() - config.lastUpdatedTimestamp;

        if (current < config.callbackTriggerCurrent) {
            config.lastUpdatedTimestamp = Timer.getFPGATimestamp();
            return;
        }

        if (triggeredDuration >= config.callbackTriggerDuration) {
            Runnable runnable = requirements.get(config);
            if (runnable != null) runnable.run();
        }
    }

    public static class CurrentLimitConfig {
        final int powerDistribution;
        final double callbackTriggerDuration, callbackTriggerCurrent;
        double lastUpdatedTimestamp;

        public CurrentLimitConfig(int powerDistributionPort, double callbackTriggerDuration, double callbackTriggerCurrent) {
            this.powerDistribution = powerDistributionPort;
            this.callbackTriggerDuration = callbackTriggerDuration;
            this.callbackTriggerCurrent = callbackTriggerCurrent;
        }
    }
}

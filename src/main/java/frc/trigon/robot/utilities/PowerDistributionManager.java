package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

public class PowerDistributionManager extends SubsystemBase {
    private final PowerDistribution PD = new PowerDistribution();
    private final HashMap<CurrentLimitConfig, Runnable> requirements = new HashMap<>();

    private final static PowerDistributionManager INSTANCE = new PowerDistributionManager();

    public static PowerDistributionManager getInstance() {
        return INSTANCE;
    }

    public void setPortRequirements(int pdPort, double triggerDuration, double triggerCurrent, Runnable callback) {
        setPortRequirements(new CurrentLimitConfig(pdPort, triggerDuration, triggerCurrent), callback);
    }

    public void setPortRequirements(CurrentLimitConfig config, Runnable callback) {
        requirements.put(config, callback);
    }
    private void checkCurrent(CurrentLimitConfig config) {
        double current = PD.getCurrent(config.pdPort);
        if (current < config.triggerCurrent) {
            config.lastOkTime = Timer.getFPGATimestamp();
            return;
        }
        double triggeredDuration = Timer.getFPGATimestamp() - config.lastOkTime;
        if (triggeredDuration >= config.triggerDuration) {
            Runnable runnable = requirements.get(config);
            if (runnable != null) runnable.run();
        }
    }

    @Override
    public void periodic() {
        requirements.keySet().forEach(this::checkCurrent);
    }

    public static class CurrentLimitConfig {
        int pdPort;
        double triggerDuration, triggerCurrent, lastOkTime;

        public CurrentLimitConfig(int pdPort, double triggerDuration, double triggerCurrent) {
            this.pdPort = pdPort;
            this.triggerDuration = triggerDuration;
            this.triggerCurrent = triggerCurrent;
        }
    }
}

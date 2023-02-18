package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class PowerDistributionManager extends SubsystemBase {
    private final PowerDistribution PD = new PowerDistribution();
    private final ArrayList<CurrentListenerConfig> requirements = new ArrayList<>();

    private final static PowerDistributionManager INSTANCE = new PowerDistributionManager();

    public static PowerDistributionManager getInstance() {
        return INSTANCE;
    }

    public void setPortRequirements(int port, double second, double maxAMP, Runnable function) {
        requirements.add(new CurrentListenerConfig(port, second, maxAMP, function));
    }

    private void checkCurrent(CurrentListenerConfig config) {
        double current = PD.getCurrent(config.pdPort);
        if (current < config.triggerCurrent){
            config.lastOkTime = Timer.getFPGATimestamp();
            return;
        }
        double triggeredDuration = Timer.getFPGATimestamp() - config.lastOkTime;
        if (triggeredDuration >= config.triggerDuration)
            config.callback.run();
    }

    @Override
    public void periodic() {
        requirements.forEach(this::checkCurrent);
    }

    static class CurrentListenerConfig {
        int pdPort;
        double triggerDuration, triggerCurrent, lastOkTime;
        Runnable callback;

        public CurrentListenerConfig(int pdPort, double triggerDuration, double triggerCurrent, Runnable callback) {
            this.pdPort = pdPort;
            this.triggerDuration = triggerDuration;
            this.triggerCurrent = triggerCurrent;
            this.callback = callback;
        }
    }
}

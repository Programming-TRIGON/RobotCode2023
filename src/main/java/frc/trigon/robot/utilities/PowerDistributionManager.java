package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class PowerDistributionManager extends SubsystemBase {

    private final static PowerDistributionManager INSTANCE = new PowerDistributionManager();

    @SuppressWarnings("WeakerAccess")
    public static PowerDistributionManager getInstance() {
        return INSTANCE;
    }

    private PowerDistribution PD = new PowerDistribution();
    private double TimerBeforeMaxAmp = 0;
    static ArrayList<PortRequirements> requirements = new ArrayList<>();

    private PowerDistributionManager() {
    }

    public static void setPortRequirements(int port, double second, double maxAMP, Runnable function) {
        requirements.add(new PortRequirements(port, second, maxAMP, function));
    }

    private void checkPort(PortRequirements requirements) {
        if (PD.getCurrent(requirements.port) >= requirements.maxAMP) {
            if (Timer.getFPGATimestamp() >= TimerBeforeMaxAmp + requirements.second) {
                requirements.function.run();
            }
        } else {
            TimerBeforeMaxAmp = Timer.getFPGATimestamp();

        }
    }

    @Override
    public void periodic() {
        requirements.forEach((n) -> checkPort(n));
    }
}

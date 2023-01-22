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
    static ArrayList<PortRequirements> requirements = new ArrayList<>();

    public static void setPortRequirements(int port, double second, double maxAMP, Runnable function) {
        double timer = 0, timerBeforeMaxAmp = 0;
        requirements.add(new PortRequirements(port, second, maxAMP, function, timer, timerBeforeMaxAmp));
    }

    private void checkPort(PortRequirements requirements) {
        if (PD.getCurrent(requirements.port) >= requirements.maxAMP) {
            requirements.timer = Timer.getFPGATimestamp();
            if (requirements.timer >= requirements.timerBeforeMaxAmp + requirements.second) {
                requirements.function.run();
            }
        } else {
            requirements.timerBeforeMaxAmp = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void periodic() {
        requirements.forEach((n) -> checkPort(n));
    }
}

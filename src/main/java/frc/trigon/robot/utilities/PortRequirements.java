package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Timer;

public class PortRequirements {

    int port;
    double second, maxAMP;
    Runnable function;
    double timer, timerBeforeMaxAmp;


    public PortRequirements(int port, double second, double maxAMP, Runnable function, double timer, double timerBeforeMaxAmp) {
        this.port = port;
        this.second = second;
        this.maxAMP = maxAMP;
        this.function = function;
        this.timer = timer;
        this.timerBeforeMaxAmp = timerBeforeMaxAmp;

    }


}

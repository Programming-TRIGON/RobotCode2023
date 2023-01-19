package frc.trigon.robot.utilities;

public class PortRequirements {

    int port;
    double second, maxAMP;
    Runnable function;


    public PortRequirements(int port, double second, double maxAMP, Runnable function) {
        this.port = port;
        this.second = second;
        this.maxAMP = maxAMP;
        this.function = function;
    }


}

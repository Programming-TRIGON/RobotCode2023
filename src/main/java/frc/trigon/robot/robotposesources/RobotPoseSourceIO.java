package frc.trigon.robot.robotposesources;

import org.littletonrobotics.junction.AutoLog;

public class RobotPoseSourceIO {
    @AutoLog
    public static class RobotPoseSourceInputs {
        public boolean hasResult = false;
        public double lastResultTimestamp = 0;
        public double[] cameraPose = new double[6];
    }

    /**
     * Updates the inputs of the robot pose source.
     *
     * @param inputs the inputs to update
     */
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
    }
}

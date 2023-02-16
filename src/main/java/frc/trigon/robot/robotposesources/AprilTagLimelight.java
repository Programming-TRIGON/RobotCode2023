package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagLimelight extends RobotPoseSource {
    private final String hostname;
    private final Limelight limelight;

    public AprilTagLimelight(String hostname) {
        super(new Transform3d());

        this.hostname = hostname;
        limelight = new Limelight(hostname);
    }

    @Override
    public Pose3d getCameraPose() {
        final Pose3d robotPose = limelight.getRobotPoseFromJsonDump();
        if (robotPose == null)
            return getLastRealPose();

        return robotPose;
    }

    @Override
    public double getLastResultTimestamp() {
        return limelight.getLastResultTimestamp();
    }

    @Override
    public String getName() {
        return hostname;
    }
}

package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.components.Limelight;

public class AprilTagLimelight extends RobotPoseSource {
    private final Limelight limelight;

    public AprilTagLimelight(String hostname) {
        super(new Transform3d());

        limelight = new Limelight(hostname);
    }

    @Override
    public Pose3d getCameraPose() {
        final Pose3d robotPose = limelight.getRobotPoseFromJsonDump();
        if (robotPose == null)
            return getLastProvidedPose();

        return robotPose;
    }

    @Override
    public double getLastResultTimestamp() {
        return limelight.getLastResultTimestamp();
    }

    @Override
    public String getName() {
        return limelight.getName();
    }
}

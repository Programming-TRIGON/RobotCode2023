package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.components.FiducialLimelight;

public class AprilTagLimelight extends RobotPoseSource {
    private final FiducialLimelight fiducialLimelight;

    public AprilTagLimelight(String hostname) {
        super(new Transform3d());

        fiducialLimelight = new FiducialLimelight(hostname);
    }

    @Override
    public Pose3d getCameraPose() {
        return fiducialLimelight.getRobotPoseFromJsonDump();
    }

    @Override
    public double getLastResultTimestamp() {
        return fiducialLimelight.getLastResultTimestamp();
    }

    @Override
    public String getName() {
        return fiducialLimelight.getName();
    }
}

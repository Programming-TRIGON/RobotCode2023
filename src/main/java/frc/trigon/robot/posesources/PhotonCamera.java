package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import io.github.oblarg.oblog.Loggable;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class PhotonCamera extends org.photonvision.PhotonCamera implements PoseSource, Loggable {
    private final PhotonPoseEstimator photonPoseEstimator;
    private double lastUpdatedTimestamp = 0;
    private Pose2d lastRealPose = new Pose2d();

    public PhotonCamera(String cameraName, Transform3d cameraToRobot) {
        super(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                PoseSourceConstants.APRIL_TAG_FIELD_LAYOUT,
                PoseSourceConstants.POSE_STRATEGY,
                this,
                cameraToRobot.inverse()
        );
    }

    @Override
    public Pose2d getLastRealPose() {
        return lastRealPose;
    }

    @Override
    public double getTimestampSeconds() {
        return getLatestResult().getTimestampSeconds();
    }

    @Override
    public boolean hasResults() {
        return true;
    }

    @Override
    public Pose2d getRobotPose() {
        final Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isEmpty())
            return null;

        lastRealPose = estimatedRobotPose.get().estimatedPose.toPose2d();
        return lastRealPose;
    }

    @Override
    public double getLastUpdatedTimestamp() {
        return lastUpdatedTimestamp;
    }

    @Override
    public void setLastUpdatedTimestamp(double timestamp) {
        lastUpdatedTimestamp = timestamp;
    }
}

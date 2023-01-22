package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraPoseSource extends PhotonCamera implements PoseSource, Loggable {
    private final Transform3d cameraToRobot;
    private double previousTimestamp = 0;

    @Config
    private double maximumTagAmbiguity = 0.2;

    public PhotonCameraPoseSource(String cameraName, Transform3d cameraToRobot) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
    }

    public PhotonCameraPoseSource(String cameraName, Transform3d cameraToRobot, double maximumTagAmbiguity) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
        this.maximumTagAmbiguity = maximumTagAmbiguity;
    }

    @Override
    public boolean canUpdate() {
        return hasNewResult() && isCurrentTagGood();
    }

    @Override
    public double getTimestampSeconds() {
        return getLatestResult().getTimestampSeconds();
    }

    @Override
    public boolean hasResults() {
        return getLatestResult().hasTargets();
    }

    @Override
    public Pose2d getRobotPose() {
        final PhotonTrackedTarget bestTarget = getLatestResult().getBestTarget();

        return getRobotPoseFromTag(bestTarget);
    }

    @Override
    public double getPreviousTimestamp() {
        return previousTimestamp;
    }

    @Override
    public void setPreviousTimestamp(double timestamp) {
        this.previousTimestamp = timestamp;
    }

    private boolean isCurrentTagGood() {
        final PhotonTrackedTarget bestTag = getLatestResult().getBestTarget();
        final int tagId = bestTag.getFiducialId();
        final double tagAmbiguity = bestTag.getPoseAmbiguity();
        final int tagsCount = PoseSourceConstants.TAG_POSES.size();

        return tagId >= 0 && tagId < tagsCount && tagAmbiguity <= maximumTagAmbiguity;
    }

    private Pose2d getRobotPoseFromTag(PhotonTrackedTarget tag) {
        final int tagId = tag.getFiducialId();
        final Transform3d cameraToTag = tag.getBestCameraToTarget();
        final Pose3d tagPose = PoseSourceConstants.TAG_POSES.get(tagId);

        return PhotonUtils.estimateFieldToRobotAprilTag(
                cameraToTag,
                tagPose,
                cameraToRobot
        ).toPose2d();
    }
}

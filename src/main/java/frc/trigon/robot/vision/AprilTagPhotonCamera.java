package frc.trigon.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagPhotonCamera extends PhotonCamera implements AprilTagCamera, Loggable {
    private final Transform3d cameraToRobot;
    private double previousTimestamp = 0;

    @Config
    private double maximumTagAmbiguity = 0.2;

    public AprilTagPhotonCamera(String cameraName, Transform3d cameraToRobot) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
    }

    public AprilTagPhotonCamera(String cameraName, Transform3d cameraToRobot, double maximumTagAmbiguity) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
        this.maximumTagAmbiguity = maximumTagAmbiguity;
    }

    @Override
    public boolean doesHaveGoodTag() {
        final PhotonTrackedTarget bestTag = getLatestResult().getBestTarget();
        final int tagId = bestTag.getFiducialId();
        final double tagAmbiguity = bestTag.getPoseAmbiguity();
        final int tagsCount = VisionConstants.TAG_POSES.size();

        return tagId >= 0 && tagId < tagsCount && tagAmbiguity <= maximumTagAmbiguity;
    }

    @Override
    public double getTimestampSeconds() {
        return getLatestResult().getTimestampSeconds();
    }

    @Override
    public boolean hasTags() {
        return this.getLatestResult().hasTargets();
    }

    @Override
    public Pose3d getRobotPose() {
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

    private Pose3d getRobotPoseFromTag(PhotonTrackedTarget tag) {
        final int tagId = tag.getFiducialId();
        final Transform3d cameraToTag = tag.getBestCameraToTarget();
        final Pose3d tagPose = VisionConstants.TAG_POSES.get(tagId);

        return PhotonUtils.estimateFieldToRobotAprilTag(
                cameraToTag,
                tagPose,
                cameraToRobot
        );
    }
}

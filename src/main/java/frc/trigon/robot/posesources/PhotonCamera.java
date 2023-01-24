package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class PhotonCamera extends org.photonvision.PhotonCamera implements PoseSource, Loggable {
    private final Transform3d cameraToRobot;
    private double lastTimestamp = 0;
    private double maximumTagAmbiguity = 0.05;

    public PhotonCamera(String cameraName, Transform3d cameraToRobot) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
    }

    public PhotonCamera(String cameraName, Transform3d cameraToRobot, double maximumTagAmbiguity) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
        this.maximumTagAmbiguity = maximumTagAmbiguity;
    }

    @Config(defaultValueNumeric = 0.05)
    public void setMaximumTagAmbiguity(double maximumTagAmbiguity) {
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
    public Pose2d getRobotPose(Rotation2d gyroAngle) {
        final PhotonTrackedTarget bestTag = getCurrentBestTag(gyroAngle);

        return getRobotPoseFromTag(bestTag);
    }

    @Override
    public double getLastTimestamp() {
        return lastTimestamp;
    }

    @Override
    public void setLastTimestamp(double timestamp) {
        this.lastTimestamp = timestamp;
    }

    private PhotonTrackedTarget getCurrentBestTag(Rotation2d currentAngle) {
        final List<PhotonTrackedTarget> tags = getLatestResult().getTargets();

        double minTagError = 0;
        PhotonTrackedTarget bestTag = getLatestResult().getBestTarget();

        for (PhotonTrackedTarget currentTag : tags) {
            final Pose2d currentTagPose = getRobotPoseFromTag(currentTag);
            final Rotation2d currentTagRotation = currentTagPose.getRotation();
            final double tagError = Math.abs(currentAngle.minus(currentTagRotation).getDegrees());

            if (tagError > minTagError) {
                minTagError = tagError;
                bestTag = currentTag;
            }
        }

        return bestTag;
    }

    private boolean isCurrentTagGood() {
        final PhotonTrackedTarget bestTag = getLatestResult().getBestTarget();

        if (bestTag == null) return false;

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

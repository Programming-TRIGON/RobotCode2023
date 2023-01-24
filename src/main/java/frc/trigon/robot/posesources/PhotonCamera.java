package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.utilities.Maths;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
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
        final List<PhotonTrackedTarget> visibleTags = getLatestResult().getTargets();
        final List<Pose2d> tagPoses = new ArrayList<>();

        for (PhotonTrackedTarget currentTag : visibleTags) {
            tagPoses.add(getBestPoseFromTag(currentTag, gyroAngle));
        }

        return getAveragePose(tagPoses);
    }

    @Override
    public double getLastTimestamp() {
        return lastTimestamp;
    }

    @Override
    public void setLastTimestamp(double timestamp) {
        this.lastTimestamp = timestamp;
    }

    private Pose2d getBestPoseFromTag(PhotonTrackedTarget tag, Rotation2d gyroAngle) {
        final Pose2d
                bestPose = getRobotPoseFromTag(tag, false),
                alternatePose = getRobotPoseFromTag(tag, true);

        final double
                bestPoseError = Math.abs(gyroAngle.minus(bestPose.getRotation()).getDegrees()),
                alternatePoseError = Math.abs(gyroAngle.minus(alternatePose.getRotation()).getDegrees());

        return bestPoseError <= alternatePoseError ? bestPose : alternatePose;
    }

    private boolean isCurrentTagGood() {
        final PhotonTrackedTarget bestTag = getLatestResult().getBestTarget();

        if (bestTag == null) return false;

        final int tagId = bestTag.getFiducialId();
        final double tagAmbiguity = bestTag.getPoseAmbiguity();
        final int tagsCount = PoseSourceConstants.TAG_POSES.size();

        return tagId >= 0 && tagId < tagsCount && tagAmbiguity <= maximumTagAmbiguity;
    }

    private Pose2d getRobotPoseFromTag(PhotonTrackedTarget tag, boolean isAlternate) {
        final int tagId = tag.getFiducialId();
        final Transform3d cameraToTag = isAlternate ? tag.getAlternateCameraToTarget() : tag.getBestCameraToTarget();
        final Pose3d tagPose = PoseSourceConstants.TAG_POSES.get(tagId);

        return PhotonUtils.estimateFieldToRobotAprilTag(
                cameraToTag,
                tagPose,
                cameraToRobot
        ).toPose2d();
    }

    private Pose2d getAveragePose(List<Pose2d> poses) {
        final List<Double>
                xValues = new ArrayList<>(),
                yValues = new ArrayList<>(),
                degreeValues = new ArrayList<>();

        for (Pose2d currentPose : poses) {
            xValues.add(currentPose.getTranslation().getX());
            yValues.add(currentPose.getTranslation().getY());
            degreeValues.add(currentPose.getRotation().getDegrees());
        }

        final double
                averageX = Maths.average(xValues),
                averageY = Maths.average(yValues),
                averageDegrees = Maths.average(degreeValues);

        return new Pose2d(averageX, averageY, Rotation2d.fromDegrees(averageDegrees));
    }

}

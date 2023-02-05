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
import java.util.function.Supplier;

public class PhotonCamera extends org.photonvision.PhotonCamera implements PoseSource, Loggable {
    private final Transform3d cameraToRobot;
    private final Supplier<Rotation2d> gyroRotationSupplier;
    private double lastUpdatedTimestamp = 0;
    private double maximumTagAmbiguity = 0.2;
    private Pose2d lastRealPose = new Pose2d();

    public PhotonCamera(String cameraName, Transform3d cameraToRobot, Supplier<Rotation2d> gyroRotationSupplier) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
        this.gyroRotationSupplier = gyroRotationSupplier;
    }

    public PhotonCamera(String cameraName, Transform3d cameraToRobot, Supplier<Rotation2d> gyroRotationSupplier, double maximumTagAmbiguity) {
        super(cameraName);

        this.cameraToRobot = cameraToRobot;
        this.gyroRotationSupplier = gyroRotationSupplier;
        this.maximumTagAmbiguity = maximumTagAmbiguity;
    }

    @Config(defaultValueNumeric = 0.2)
    public void setMaximumTagAmbiguity(double maximumTagAmbiguity) {
        this.maximumTagAmbiguity = maximumTagAmbiguity;
    }

    @Override
    public Pose2d getLastRealPose() {
        return lastRealPose;
    }

    @Override
    public boolean canUpdate() {
        return hasNewResult() && (isGoodTag(getLatestResult().getBestTarget()) || getLatestResult().getTargets().size() > 1);
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
        if (!hasResults()) return new Pose2d();

        final List<PhotonTrackedTarget> visibleTags = getLatestResult().getTargets();
        if (visibleTags.size() == 0) return new Pose2d();

        final List<Pose2d> tagPoses = new ArrayList<>();

        for (PhotonTrackedTarget currentTag : visibleTags) {
            final Pose2d estimatedPoseFromTag = getEstimatedRobotPoseFromTag(currentTag);

            if (estimatedPoseFromTag == null) continue;

            tagPoses.add(estimatedPoseFromTag);
        }

        final Pose2d averagePose = getAveragePose(tagPoses);
        lastRealPose = averagePose;

        return averagePose;
    }

    @Override
    public double getLastUpdatedTimestamp() {
        return lastUpdatedTimestamp;
    }

    @Override
    public void setLastUpdatedTimestamp(double timestamp) {
        this.lastUpdatedTimestamp = timestamp;
    }

    private Pose2d getEstimatedRobotPoseFromTag(PhotonTrackedTarget tag) {
        if (!doesHaveWantedId(tag)) return null;

        if (!doesHaveAlternatePose(tag)) return getBestRobotPoseFromTag(tag);

        final Pose2d
                bestPose = getBestRobotPoseFromTag(tag),
                alternatePose = getAlternateRobotPoseFromTag(tag);

        if (alternatePose == null) return bestPose;
        if (bestPose == null) return alternatePose;

        final Rotation2d gyroRotation = gyroRotationSupplier.get();

        final double
                bestPoseError = Math.abs(gyroRotation.minus(bestPose.getRotation()).getDegrees()),
                alternatePoseError = Math.abs(gyroRotation.minus(alternatePose.getRotation()).getDegrees());

        return bestPoseError <= alternatePoseError ? bestPose : alternatePose;
    }

    private Pose2d getAlternateRobotPoseFromTag(PhotonTrackedTarget tag) {
        final int tagId = tag.getFiducialId();
        final Transform3d cameraToTag = tag.getAlternateCameraToTarget();

        if (tagId > PoseSourceConstants.TAG_POSES.size()) return null;

        final Pose3d tagPose = PoseSourceConstants.TAG_POSES.get(tagId);

        return PhotonUtils.estimateFieldToRobotAprilTag(
                cameraToTag,
                tagPose,
                cameraToRobot
        ).toPose2d();
    }

    private Pose2d getBestRobotPoseFromTag(PhotonTrackedTarget tag) {
        final int tagId = tag.getFiducialId();
        final Transform3d cameraToTag = tag.getBestCameraToTarget();

        if (tagId > PoseSourceConstants.TAG_POSES.size()) return null;

        final Pose3d tagPose = PoseSourceConstants.TAG_POSES.get(tagId);

        return PhotonUtils.estimateFieldToRobotAprilTag(
                cameraToTag,
                tagPose,
                cameraToRobot
        ).toPose2d();
    }

    private boolean isGoodTag(PhotonTrackedTarget tag) {
        if (tag == null) return false;

        final double tagAmbiguity = tag.getPoseAmbiguity();

        return doesHaveWantedId(tag) && tagAmbiguity <= maximumTagAmbiguity;
    }

    private boolean doesHaveWantedId(PhotonTrackedTarget tag) {
        final int tagId = tag.getFiducialId();
        final int tagsCount = PoseSourceConstants.TAG_POSES.size();

        return tagId > 0 && tagId < tagsCount;
    }

    private boolean doesHaveAlternatePose(PhotonTrackedTarget tag) {
        final Transform3d alternateCameraToTarget = tag.getAlternateCameraToTarget();
        final Pose2d alternateCameraToTargetPose = new Pose3d().transformBy(alternateCameraToTarget).toPose2d();

        final double
                x = alternateCameraToTargetPose.getX(),
                y = alternateCameraToTargetPose.getY();

        return
                (Math.abs(x) > PoseSourceConstants.POSE_TOLERANCE ||
                        Math.abs(y) > PoseSourceConstants.POSE_TOLERANCE);
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

package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import io.github.oblarg.oblog.Loggable;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class PhotonCamera extends AbsolutePoseSource implements Loggable {
    private final org.photonvision.PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public PhotonCamera(String cameraName, Transform3d cameraToRobotCenter) {
        photonCamera = new org.photonvision.PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                PoseSourceConstants.APRIL_TAG_FIELD_LAYOUT,
                PoseSourceConstants.POSE_STRATEGY,
                photonCamera,
                cameraToRobotCenter.inverse()
        );
    }

    @Override
    public double getLastResultTimestamp() {
        return photonCamera.getLatestResult().getTimestampSeconds();
    }

    @Override
    public Pose2d getRobotPose() {
        final Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isEmpty())
            return getLastRealPose();

        setLastRealPose(estimatedRobotPose.get().estimatedPose.toPose2d());
        return getLastRealPose();
    }

    @Override
    public String getName() {
        return photonCamera.getName();
    }
}

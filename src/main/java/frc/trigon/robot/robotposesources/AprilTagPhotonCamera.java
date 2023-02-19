package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class AprilTagPhotonCamera extends RobotPoseSource {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public AprilTagPhotonCamera(String cameraName, Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
        photonCamera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                PoseSourceConstants.APRIL_TAG_FIELD_LAYOUT,
                PoseSourceConstants.POSE_STRATEGY,
                photonCamera,
                new Transform3d()
        );
    }

    @Override
    public double getLastResultTimestamp() {
        return photonCamera.getLatestResult().getTimestampSeconds();
    }

    @Override
    public Pose3d getCameraPose() {
        final Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isEmpty())
            return null;

        return estimatedRobotPose.get().estimatedPose;
    }

    @Override
    public String getName() {
        return photonCamera.getName();
    }
}

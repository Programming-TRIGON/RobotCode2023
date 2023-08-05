package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.utilities.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import java.util.Optional;

public class AprilTagPhotonCamera extends RobotPoseSourceIO {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    protected AprilTagPhotonCamera(String cameraName, Transform3d cameraToRobotCenter) {
        photonCamera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                PoseSourceConstants.APRIL_TAG_FIELD_LAYOUT,
                PoseSourceConstants.PRIMARY_POSE_STRATEGY,
                photonCamera,
                cameraToRobotCenter.inverse()
        );

        photonPoseEstimator.setMultiTagFallbackStrategy(PoseSourceConstants.SECONDARY_POSE_STRATEGY);
        if (PoseSourceConstants.SECONDARY_POSE_STRATEGY == PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
            photonPoseEstimator.setReferencePose(new Pose3d());
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = photonCamera.getLatestResult().hasTargets();
        if (inputs.hasResult)
            inputs.cameraPose = RobotPoseSource.pose3dToDoubleArray(getCameraPose());
        inputs.lastResultTimestamp = photonCamera.getLatestResult().getTimestampSeconds();
    }

    private Pose3d getCameraPose() {
        if (PoseSourceConstants.SECONDARY_POSE_STRATEGY == PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
            photonPoseEstimator.setReferencePose(PoseEstimator.getInstance().getCurrentPose());

        final Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isEmpty())
            return null;

        return estimatedRobotPose.get().estimatedPose;
    }
}

package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.utilities.PhotonPoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import java.util.Optional;

public class AprilTagPhotonCamera extends RobotPoseSource {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public AprilTagPhotonCamera(String cameraName, Transform3d cameraToRobotCenter) {
        super(new Transform3d());
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
        if (inputs.name.equals(""))
            inputs.name = photonCamera.getName();

        inputs.hasResult = photonCamera.getLatestResult().hasTargets();
        inputs.cameraPose = getCameraPose();
        inputs.lastResultTimestamp = photonCamera.getLatestResult().getTimestampSeconds();
    }
    private Pose3d getCameraPose() {
        if (PoseSourceConstants.SECONDARY_POSE_STRATEGY == PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
            photonPoseEstimator.setReferencePose(PoseEstimator.getInstance().getCurrentPose());

        final Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isEmpty())
            return null;

        var x= estimatedRobotPose.get().estimatedPose;
        Logger.getInstance().recordOutput("espos/x", x.getX());
        Logger.getInstance().recordOutput("espos/y", x.getY());
        Logger.getInstance().recordOutput("espos/z", x.getZ());
        Logger.getInstance().recordOutput("espos/rotX", x.getRotation().getX());
        Logger.getInstance().recordOutput("espos/rotY", x.getRotation().getY());
        Logger.getInstance().recordOutput("espos/rotZ", x.getRotation().getZ());

        return estimatedRobotPose.get().estimatedPose;
    }
}

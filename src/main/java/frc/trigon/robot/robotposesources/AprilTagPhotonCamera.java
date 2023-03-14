package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.utilities.PhotonPoseEstimator;
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
    public double getLastResultTimestamp() {
        return photonCamera.getLatestResult().getTimestampSeconds();
    }

    @Override
    public String getName() {
        return photonCamera.getName();
    }

    @Override
    protected boolean hasResult() {
        return photonCamera.getLatestResult().hasTargets();
    }

    @Override
    protected Pose3d getCameraPose() {
        if (PoseSourceConstants.SECONDARY_POSE_STRATEGY == PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
            photonPoseEstimator.setReferencePose(PoseEstimator.getInstance().getCurrentPose());

        final Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isEmpty())
            return null;

        var x= estimatedRobotPose.get().estimatedPose;
        SmartDashboard.putNumber("espos/x", x.getX());
        SmartDashboard.putNumber("espos/y", x.getY());
        SmartDashboard.putNumber("espos/z", x.getZ());
        SmartDashboard.putNumber("espos/rotX", x.getRotation().getX());
        SmartDashboard.putNumber("espos/rotY", x.getRotation().getY());
        SmartDashboard.putNumber("espos/rotZ", x.getRotation().getZ());

        return estimatedRobotPose.get().estimatedPose;
    }
}

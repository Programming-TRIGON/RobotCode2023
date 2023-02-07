package frc.trigon.robot.posesources;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PoseSourceConstants {
    public static final List<Pose3d> TAG_POSES = new ArrayList<>();
    static final PhotonPoseEstimator.PoseStrategy POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS;
    static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = null;

    static {
        try {
            APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            final List<AprilTag> aprilTags = APRIL_TAG_FIELD_LAYOUT.getTags();

            TAG_POSES.add(null);

            for (AprilTag aprilTag : aprilTags)
                TAG_POSES.add(aprilTag.pose);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

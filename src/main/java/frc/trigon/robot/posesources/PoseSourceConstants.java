package frc.trigon.robot.posesources;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

public class PoseSourceConstants {
    public static final HashMap<Integer, Pose3d> TAGS_ID_TO_POSE = new HashMap<>();
    static final PhotonPoseEstimator.PoseStrategy POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS;
    static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = null;

    static {
        try {
            APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            final List<AprilTag> aprilTags = APRIL_TAG_FIELD_LAYOUT.getTags();

            for (AprilTag aprilTag : aprilTags)
                TAGS_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

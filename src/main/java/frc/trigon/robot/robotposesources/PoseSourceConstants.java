package frc.trigon.robot.robotposesources;

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
    static final PhotonPoseEstimator.PoseStrategy
            PRIMARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
            SECONDARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
    static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = null;

    static {
        try {
            APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            final List<AprilTag> aprilTags = APRIL_TAG_FIELD_LAYOUT.getTags();

            for (AprilTag aprilTag : aprilTags)
                TAGS_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

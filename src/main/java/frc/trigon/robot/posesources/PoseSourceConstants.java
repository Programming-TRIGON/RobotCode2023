package frc.trigon.robot.posesources;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PoseSourceConstants {
    public static final List<Pose3d> TAG_POSES = getTagPoses();
    static final double POSE_TOLERANCE = 0.5;

    private static List<Pose3d> getTagPoses() {
        try {
            final AprilTagFieldLayout layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            final List<AprilTag> aprilTags = layout.getTags();
            final List<Pose3d> tagPoses = new ArrayList<>();

            for (AprilTag aprilTag : aprilTags) {
                if (aprilTag.pose != null) {
                    tagPoses.add(aprilTag.pose);
                }
            }

            tagPoses.add(0, null);

            return tagPoses;
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

package frc.trigon.robot.posesources;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PoseSourceConstants {
    public static final List<Pose3d> TAG_POSES = new ArrayList<>();
    static final double POSE_TOLERANCE = 0.5;

    static {
        try {
            final AprilTagFieldLayout layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            final List<AprilTag> aprilTags = layout.getTags();

            for (AprilTag aprilTag : aprilTags) {
                TAG_POSES.add(aprilTag.pose);
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

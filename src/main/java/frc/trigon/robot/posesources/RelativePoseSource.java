package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 */
public abstract class RelativePoseSource extends PoseSource {
    private Pose2d relativePose = new Pose2d();

    /**
     * Sets the relative pose of the pose source.
     *
     * @param pose the pose to set
     */
    public void setRelativePose(Pose2d pose) {
        relativePose = pose;
    }

    /**
     * @return the relative pose of the pose source, as a Transform2d
     */
    Transform2d getPoseToRelativePoseTransform() {
        return pose2dToTransform2d(relativePose);
    }

    private Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }
}

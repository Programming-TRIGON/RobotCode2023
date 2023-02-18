package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 * UNTESTED MATH
 */
@Deprecated
public abstract class RelativeRobotPoseSource extends RobotPoseSource {
    private Transform2d whenWasReset = new Transform2d();

    protected RelativeRobotPoseSource(Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
    }

    @Override
    public Pose2d getRobotPose() {
        return transform2dToPose2d(whenWasReset.plus(pose2dToTransform2d(super.getRobotPose())));

    }

    /**
     * Sets the relative pose of the pose source.
     *
     * @param pose the pose to set
     */
    public void setRelativePose(Pose2d pose) {
        whenWasReset = pose2dToTransform2d(super.getRobotPose()).inverse().plus(pose2dToTransform2d(pose));
    }

    private Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    private Pose2d transform2dToPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }
}

package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 */
public abstract class RelativeRobotPoseSource extends RobotPoseSource {
    private Transform2d poseToRelativePose = new Transform2d();

    protected RelativeRobotPoseSource(Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
    }

    @Override
    public Pose2d getRobotPose() {
        return super.getRobotPose().plus(poseToRelativePose);
    }

    /**
     * Sets the relative pose of the pose source.
     *
     * @param pose the pose to set
     */
    public void setRelativePose(Pose2d pose) {
        poseToRelativePose = pose2dToTransform2d(pose).inverse();
    }

    private Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }
}

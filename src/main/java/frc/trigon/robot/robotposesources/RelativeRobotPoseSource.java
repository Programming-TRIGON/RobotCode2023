package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 */
public abstract class RelativeRobotPoseSource extends RobotPoseSource {
    private Pose2d relativePose = new Pose2d();

    protected RelativeRobotPoseSource(Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
    }

    @Override
    public Pose2d getRobotPose() {
        return subtractPose(super.getRobotPose(), relativePose);
    }

    /**
     * Sets the relative pose of the pose source.
     * This should be the current pose of the robot.
     *
     * @param pose the pose to set
     */
    public void setRelativePose(Pose2d pose) {
        relativePose = subtractPose(super.getRobotPose(), pose);
    }

    private Pose2d subtractPose(Pose2d pose, Pose2d toSubtract) {
        return new Pose2d(
                pose.getTranslation().minus(toSubtract.getTranslation()),
                pose.getRotation().minus(toSubtract.getRotation())
        );
    }

    private Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    private Pose2d transform2dToPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }
}
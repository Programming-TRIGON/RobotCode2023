package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 */
public abstract class RelativeRobotPoseSource extends RobotPoseSource {
    private Transform2d whenWasReset = new Transform2d();

    protected RelativeRobotPoseSource(Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
    }

    @Override
    public Pose2d getRobotPose() {
        return new Pose2d(
                super.getRobotPose().getTranslation().minus(whenWasReset.getTranslation()),
                super.getRobotPose().getRotation().minus(whenWasReset.getRotation())
        );
    }

    /**
     * Sets the relative pose of the pose source.
     *rr
     * @param pose the pose to set
     */
    public void setRelativePose(Pose2d pose) {
//        whenWasReset = pose2dToTransform2d(pose.plus(pose2dToTransform2d(super.getRobotPose())));
//        whenWasReset = super.getRobotPose().minus(pose);
        whenWasReset = new Transform2d(
                super.getRobotPose().getTranslation().minus(pose.getTranslation()),
                super.getRobotPose().getRotation().minus(pose.getRotation())
        );
        System.out.println("when was reset: " + whenWasReset.toString());
    }

    private Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    private Pose2d transform2dToPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }
}
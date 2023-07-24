package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.Robot;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 */
public class RelativeRobotPoseSource extends RobotPoseSource {
    private Pose2d relativePose = new Pose2d();

    public RelativeRobotPoseSource(PoseSourceConstants.RelativeRobotPoseSourceType relativeRobotPoseSourceType, String name, Transform3d cameraToRobotCenter) {
        super(new RobotPoseSourceIO(), name, cameraToRobotCenter);
        setRobotPoseSourceIO(generateIO(relativeRobotPoseSourceType));
    }

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

    private RobotPoseSourceIO generateIO(PoseSourceConstants.RelativeRobotPoseSourceType relativeRobotPoseSourceType) {
        if (!Robot.IS_REAL)
            return new RobotPoseSourceIO();

        switch (relativeRobotPoseSourceType) {
            case T265:
                return new T265(name);
            default:
                return new RobotPoseSourceIO();
        }
    }
}
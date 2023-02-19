package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public abstract class RobotPoseSource {
    private final Transform3d cameraToRobotCenter;
    private double lastUpdatedTimestamp;
    private Pose3d lastProvidedPose = new Pose3d();

    protected RobotPoseSource(Transform3d cameraToRobotCenter) {
        this.cameraToRobotCenter = cameraToRobotCenter;
    }

    /**
     * @return if the current timestamp is not the same as the timestamp from the last call to this method
     */
    public boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    /**
     * @return the robot's estimated pose
     */
    public Pose2d getRobotPose() {
        lastProvidedPose = getCameraPose();
        return lastProvidedPose.plus(cameraToRobotCenter).toPose2d();
    }

    /**
     * @return the last pose the pose source has provided
     */
    protected Pose3d getLastProvidedPose() {
        return lastProvidedPose;
    }

    /**
     * @return the last result's timestamp
     */
    public abstract double getLastResultTimestamp();

    /**
     * @return the pose source's name
     */
    public abstract String getName();

    /**
     * @return the camera's pose, according to the pose source
     */
    protected abstract Pose3d getCameraPose();
}

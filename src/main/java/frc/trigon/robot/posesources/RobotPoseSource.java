package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * A pose source is a class that provides the robot's pose.
 */
public abstract class RobotPoseSource {
    private final Transform3d cameraToRobotCenter;
    private double lastUpdatedTimestamp;
    private Pose3d lastRealPose = new Pose3d();

    protected RobotPoseSource(Transform3d cameraToRobotCenter) {
        this.cameraToRobotCenter = cameraToRobotCenter;
    }

    /**
     * @return if the current timestamp is not the same as the last timestamp
     */
    public boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    /**
     * @return the robot's best estimated pose, according to the pose source
     */
    public Pose2d getRobotPose() {
        return getCameraPose().plus(cameraToRobotCenter).toPose2d();
    }

    /**
     * @return the camera to robot center transform
     */
    protected Transform3d getCameraToRobotCenter() {
        return cameraToRobotCenter;
    }

    /**
     * @return the last valid robot pose the pose source has provided
     */
    protected Pose3d getLastRealPose() {
        return lastRealPose;
    }

    /**
     * Sets the last valid robot pose the pose source has provided.
     *
     * @param pose the pose to set
     */
    protected void setLastRealPose(Pose3d pose) {
        lastRealPose = pose;
    }

    /**
     * @return the camera's pose
     */
    protected abstract Pose3d getCameraPose();

    /**
     * @return the last result's timestamp
     */
    public abstract double getLastResultTimestamp();

    /**
     * @return the pose source's name
     */
    public abstract String getName();
}

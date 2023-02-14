package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A pose source is a class that provides the robot's pose.
 */
public abstract class PoseSource {
    private double lastUpdatedTimestamp;
    private Pose2d lastRealPose = new Pose2d();

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
     * @return the last valid robot pose the pose source has provided
     */
    Pose2d getLastRealPose() {
        return lastRealPose;
    }

    /**
     * Sets the last valid robot pose the pose source has provided.
     *
     * @param pose the pose to set
     */
    void setLastRealPose(Pose2d pose) {
        lastRealPose = pose;
    }

    /**
     * Sets the current pose of the robot.
     * This may not be used by all pose sources.
     *
     * @param pose the current pose of the robot
     */
    public abstract void setCurrentPose(Pose2d pose);

    /**
     * @return the robot's best estimated pose, according to the pose source
     */
    public abstract Pose2d getRobotPose();

    /**
     * @return the last result's timestamp
     */
    public abstract double getLastResultTimestamp();

    /**
     * @return the pose source's name
     */
    public abstract String getName();
}

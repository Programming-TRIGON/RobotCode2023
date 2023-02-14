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
        if (lastUpdatedTimestamp == getTimestampSeconds())
            return false;

        lastUpdatedTimestamp =  getTimestampSeconds();
        return true;
    }

    /**
     * @return the last robot pose the pose source has provided, that went through pose validation checks
     */
    Pose2d getLastRealPose() {
        return lastRealPose;
    }

    /**
     * Sets the last robot pose the pose source has provided, that went through validation checks.
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
     * @return the current timestamp in seconds
     */
    public abstract double getTimestampSeconds();

    /**
     * @return the pose source's name
     */
    public abstract String getName();
}

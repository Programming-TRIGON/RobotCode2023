package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A pose source is a class that provides the robot's pose.
 */
public interface PoseSource {
    /**
     * @return whether there are new results since the last call to this method
     */
    default boolean hasNewResult() {
        if (getLastUpdatedTimestamp() == getTimestampSeconds())
            return false;

        setLastUpdatedTimestamp(getTimestampSeconds());
        return hasResults();
    }

    /**
     * Sets the current pose of the robot.
     * This may not be used by all pose sources.
     *
     * @param pose the current pose of the robot
     */
    void setCurrentPose(Pose2d pose);

    /**
     * @return the last robot pose the pose source has provided, that went through pose validation checks
     */
    Pose2d getLastRealPose();

    /**
     * @return true if the pose source has results, false otherwise
     */
    boolean hasResults();

    /**
     * @return the robot's best estimated pose, according to the pose source
     */
    Pose2d getRobotPose();

    /**
     * @return the current timestamp in seconds
     */
    double getTimestampSeconds();

    /**
     * @return the previous timestamp in seconds
     */
    double getLastUpdatedTimestamp();

    /**
     * Sets the previous timestamp.
     */
    void setLastUpdatedTimestamp(double timestamp);

    /**
     * @return the pose source's name
     */
    String getName();
}

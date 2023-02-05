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
        if (getLastUpdatedTimestamp() == getTimestampSeconds()) return false;

        setLastUpdatedTimestamp(getTimestampSeconds());
        return hasResults();
    }

    /**
     * @return the last "real" pose provided by the pose source. Real meaning a pose that wasn't null or defaulted.
     */
    Pose2d getLastRealPose();

    /**
     * @return true if the pose source is ready to provide a pose, false otherwise
     */
    boolean canUpdate();

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

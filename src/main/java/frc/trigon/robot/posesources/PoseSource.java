package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;

public interface PoseSource {
    /**
     * @return whether there are new results since the last call to this method
     */
    default boolean hasNewResult() {
        if (getLastTimestamp() == getTimestampSeconds()) return false;

        setLastTimestamp(getTimestampSeconds());
        return hasResults();
    }

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
    double getLastTimestamp();

    /**
     * Sets the previous timestamp.
     */
    void setLastTimestamp(double timestamp);

    /**
     * @return the pose source's name
     */
    String getName();
}

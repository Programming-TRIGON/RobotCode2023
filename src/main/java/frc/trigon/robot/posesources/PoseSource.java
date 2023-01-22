package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;

public interface PoseSource {
    /**
     * @return true if the previous timestamp is not the current timestamp, and if there are results. false otherwise
     */
    default boolean hasNewResult() {
        if (getPreviousTimestamp() == getTimestampSeconds()) return false;

        setPreviousTimestamp(getTimestampSeconds());
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
     * @return the current timestamp in seconds
     */
    double getTimestampSeconds();

    /**
     * @return the robot's pose, or null if there's no result
     */
    Pose2d getRobotPose();

    /**
     * @return the previous timestamp in seconds
     */
    double getPreviousTimestamp();

    /**
     * Sets the previous timestamp.
     */
    void setPreviousTimestamp(double timestamp);

    /**
     * @return the camera's name
     */
    String getName();
}

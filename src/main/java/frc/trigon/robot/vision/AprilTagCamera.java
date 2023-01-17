package frc.trigon.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagCamera {
    /**
     * @return true if the previous timestamp is not the current timestamp, and if the camera has a tag. false otherwise
     */
    default boolean hasNewResult() {
        if (getPreviousTimestamp() == getTimestampSeconds()) return false;

        setPreviousTimestamp(getTimestampSeconds());
        return hasTags();
    }

    /**
     * @return true if the visible tag is a good target, false otherwise
     */
    boolean doesHaveGoodTag();

    /**
     * @return the current timestamp in seconds
     */
    double getTimestampSeconds();

    /**
     * @return true if the camera has visible tags, false otherwise
     */
    boolean hasTags();

    /**
     * @return the robot's pose, or null if the camera doesn't have a good tag
     */
    Pose3d getRobotPose();

    /**
     * @return the previous timestamp in seconds
     */
    double getPreviousTimestamp();

    /**
     * Sets the previous timestamp.
     */
    void setPreviousTimestamp(double timestamp);
}

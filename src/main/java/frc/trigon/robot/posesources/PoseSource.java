package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
     * @return the current timestamp in seconds
     */
    double getTimestampSeconds();

    /**
     * @param gyroAngle the robot's current gyro angle, so that it would get the best pose relative to the current angle
     * @return the robot's best estimated pose
     */
    Pose2d getRobotPose(Rotation2d gyroAngle);

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

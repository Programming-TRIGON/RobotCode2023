package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class RobotPoseSource extends SubsystemBase {
    private final RobotPoseSourceInputsAutoLogged robotPoseSourceInputs = new RobotPoseSourceInputsAutoLogged();
    private final Transform3d cameraToRobotCenter;
    private double lastUpdatedTimestamp;
    private Pose2d lastRobotPose = new Pose2d();

    public RobotPoseSource(Transform3d cameraToRobotCenter) {
        this.cameraToRobotCenter = cameraToRobotCenter;
    }

    @Override
    public void periodic() {
        updateInputs(robotPoseSourceInputs);
    }

    /**
     * Updates the inputs of the robot pose source.
     *
     * @param inputs the inputs to update
     */
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
    }

    /**
     * @return whether the pose source has a result and that the last updated timestamp is not the current one
     */
    public boolean hasNewResult() {
        return isNewTimestamp() && robotPoseSourceInputs.hasResult;
    }

    /**
     * @return the robot's estimated pose
     */
    public Pose2d getRobotPose() {
        final Pose3d cameraPose = robotPoseSourceInputs.cameraPose;
        if (cameraPose == null)
            return lastRobotPose;
        lastRobotPose = new Pose3d(
                cameraPose.getTranslation().plus(cameraToRobotCenter.getTranslation()),
                cameraPose.getRotation().plus(cameraToRobotCenter.getRotation())
        ).toPose2d();
        return lastRobotPose;
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    /**
     * @return the last result's timestamp
     */
    public double getLastResultTimestamp() {
        return robotPoseSourceInputs.lastResultTimestamp;
    }

    /**
     * @return the pose source's name
     */
    public String getName() {
        return robotPoseSourceInputs.name;
    }

    @AutoLog
    public static class RobotPoseSourceInputs {
        public Pose3d cameraPose = new Pose3d();
        public boolean hasResult = false;
        public double lastResultTimestamp = 0;
        public String name = "";
    }
}

package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.*;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 */
public class RelativeRobotPoseSource extends RobotPoseSource {
    private final Transform2d cameraToRobotCenter;
    private Pose2d difference = new Pose2d();

    public RelativeRobotPoseSource(PoseSourceConstants.RobotPoseSourceType relativeRobotPoseSourceType, String name, Transform2d cameraToRobotCenter) {
        super(relativeRobotPoseSourceType, name, new Transform3d());
        this.cameraToRobotCenter = cameraToRobotCenter;
    }

    @Override
    public Pose2d getRobotPose() {
        final Pose2d startRelativePose = doubleArrayToPose3d(robotPoseSourceInputs.cameraPose).toPose2d();
        if (startRelativePose == null)
            return lastRobotPose;
        final Translation2d rotatedTranslation = startRelativePose.getTranslation().rotateBy(difference.getRotation().unaryMinus());
        final Translation2d subtractedTranslation = rotatedTranslation.minus(difference.getTranslation());
        final Rotation2d subtractedAngle = startRelativePose.getRotation().minus(difference.getRotation());
        final Pose2d cameraPose = new Pose2d(subtractedTranslation, subtractedAngle);

        lastRobotPose = cameraPose.transformBy(cameraToRobotCenter);
        return lastRobotPose;
    }

    /**
     * Sets the relative pose of the pose source.
     * This should be the current pose of the robot.
     *
     * @param pose the pose to set
     */
    public void setRelativePose(Pose2d pose) {
        final Pose2d startRelativePose = doubleArrayToPose3d(robotPoseSourceInputs.cameraPose).toPose2d();
        final Rotation2d angleDifference = startRelativePose.getRotation().minus(pose.getRotation());
        final Translation2d rotatedTranslation = startRelativePose.getTranslation().rotateBy(angleDifference.unaryMinus());
        final Translation2d translationDifference = rotatedTranslation.minus(pose.getTranslation());

        difference = new Pose2d(translationDifference, angleDifference);
    }
}
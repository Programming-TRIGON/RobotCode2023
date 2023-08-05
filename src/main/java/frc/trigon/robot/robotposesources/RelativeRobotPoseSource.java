package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.Robot;

/**
 * A pose source that provides the robot's pose, relative to a given pose.
 */
public class RelativeRobotPoseSource extends RobotPoseSource {
    private Pose2d difference = new Pose2d();

    public RelativeRobotPoseSource(PoseSourceConstants.RelativeRobotPoseSourceType relativeRobotPoseSourceType, String name, Transform3d cameraToRobotCenter) {
        super(new RobotPoseSourceIO(), name, cameraToRobotCenter);
        setRobotPoseSourceIO(generateIO(relativeRobotPoseSourceType));
    }

    @Override
    public Pose2d getRobotPose() {
        final Pose2d startRelativePose = super.getRobotPose();
        final Translation2d rotatedTranslation = startRelativePose.getTranslation().rotateBy(difference.getRotation().unaryMinus());
        final Translation2d subtractedTranslation = rotatedTranslation.minus(difference.getTranslation());
        final Rotation2d subtractedAngle = startRelativePose.getRotation().minus(difference.getRotation());

        return new Pose2d(subtractedTranslation, subtractedAngle);
    }

    /**
     * Sets the relative pose of the pose source.
     * This should be the current pose of the robot.
     *
     * @param pose the pose to set
     */
    public void setRelativePose(Pose2d pose) {
        final Pose2d startRelativePose = super.getRobotPose();
        final Rotation2d angleDifference = startRelativePose.getRotation().minus(pose.getRotation());
        final Translation2d rotatedTranslation = startRelativePose.getTranslation().rotateBy(angleDifference.unaryMinus());
        final Translation2d translationDifference = rotatedTranslation.minus(pose.getTranslation());

        difference = new Pose2d(translationDifference, angleDifference);
    }

    private RobotPoseSourceIO generateIO(PoseSourceConstants.RelativeRobotPoseSourceType relativeRobotPoseSourceType) {
        if (!Robot.IS_REAL)
            return new RobotPoseSourceIO();
        if (relativeRobotPoseSourceType == PoseSourceConstants.RelativeRobotPoseSourceType.T265)
            return new T265(name);

        return new RobotPoseSourceIO();
    }
}
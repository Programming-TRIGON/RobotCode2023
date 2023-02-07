package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.posesources.PoseSource;
import frc.trigon.robot.posesources.PoseSourceConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;


public class PoseEstimator extends SubsystemBase implements Loggable {
    private final static PoseEstimator INSTANCE = new PoseEstimator();

    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    @Log
    private final Field2d field = new Field2d();

    private PoseEstimator() {
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.KINEMATICS,
                swerve.getHeading(),
                swerve.getModulePositions(),
                new Pose2d(),
                PoseEstimatorConstants.STATES_AMBIGUITY,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );

        addAprilTagsToFieldWidget();
    }

    public static PoseEstimator getInstance() {
        return INSTANCE;
    }


    @Override
    public void periodic() {
        updatePoseEstimator();
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to
     */
    public void resetPose(Pose2d currentPose) {
        setGyroHeadingAndWaitUntilUpdate(currentPose.getRotation());

        swerveDrivePoseEstimator.resetPosition(
                swerve.getHeading(),
                swerve.getModulePositions(),
                currentPose
        );
    }

    /**
     * @return the current estimated pose of the robot
     */
    public Pose2d getCurrentPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    private void updatePoseEstimator() {
        attemptToUpdateWithPoseSources();
        updatePoseEstimatorStates();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithPoseSources() {
        for (PoseSource poseSource : PoseEstimatorConstants.POSE_SOURCES) {
            if (poseSource.getRobotPose() == null)
                continue;

            updateWithPoseSource(poseSource);
        }
    }

    private void updateWithPoseSource(PoseSource poseSource) {
        final Pose2d robotPose = poseSource.getRobotPose();

        swerveDrivePoseEstimator.addVisionMeasurement(
                robotPose,
                poseSource.getTimestampSeconds()
        );

        field.getObject(poseSource.getName()).setPose(robotPose);
    }

    private void updatePoseEstimatorStates() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getModulePositions());
    }

    private void addAprilTagsToFieldWidget() {
        final List<Pose3d> tagPoses = PoseSourceConstants.TAG_POSES;
        final int tagsCount = tagPoses.size();

        for (int i = 1; i < tagsCount; i++)
            field.getObject("Tag " + i).setPose(tagPoses.get(i).toPose2d());
    }

    private void setGyroHeadingAndWaitUntilUpdate(Rotation2d heading) {
        swerve.setHeading(heading);
        waitUntilGyroUpdate();
    }

    private void waitUntilGyroUpdate() {
        try {
            Thread.sleep(PoseEstimatorConstants.GYRO_UPDATE_DELAY_MS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}

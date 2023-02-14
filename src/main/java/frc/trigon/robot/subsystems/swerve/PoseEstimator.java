package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.posesources.PoseSource;
import frc.trigon.robot.posesources.PoseSourceConstants;
import frc.trigon.robot.posesources.RelativePoseSource;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.HashMap;


public class PoseEstimator extends SubsystemBase implements Loggable {
    private final static PoseEstimator INSTANCE = new PoseEstimator();

    private final Swerve swerve = RobotContainer.SWERVE;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    @Log
    private final Field2d field = new Field2d();
    private PoseSource[] poseSources = {};

    private PoseEstimator() {
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
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

        setRelativePose(currentPose);
    }

    /**
     * @return the current estimated pose of the robot
     */
    public Pose2d getCurrentPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * Sets the pose sources to use in the pose estimator.
     *
     * @param poseSources the pose sources to use
     */
    public void setPoseSources(PoseSource... poseSources) {
        this.poseSources = poseSources;
    }

    private void setRelativePose(Pose2d pose) {
        for (PoseSource poseSource : poseSources) {
            if (!(poseSource instanceof RelativePoseSource))
                continue;

            final RelativePoseSource relativePoseSource = (RelativePoseSource) poseSource;
            relativePoseSource.setRelativePose(pose);
        }
    }

    private void updatePoseEstimator() {
        attemptToUpdateWithPoseSources();
        updatePoseEstimatorStates();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithPoseSources() {
        for (PoseSource poseSource : poseSources) {
            if (poseSource.isNewTimestamp())
                updateWithPoseSource(poseSource);
        }
    }

    private void updateWithPoseSource(PoseSource poseSource) {
        final Pose2d robotPose = poseSource.getRobotPose();

        swerveDrivePoseEstimator.addVisionMeasurement(
                robotPose,
                poseSource.getLastResultTimestamp()
        );

        field.getObject(poseSource.getName()).setPose(robotPose);
    }

    private void updatePoseEstimatorStates() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getModulePositions());
    }

    private void addAprilTagsToFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = PoseSourceConstants.TAGS_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet())
            field.getObject("Tag " + currentID).setPose(tagsIdToPose.get(currentID).toPose2d());
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

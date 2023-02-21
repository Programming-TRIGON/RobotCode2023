package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.robotposesources.PoseSourceConstants;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A class that estimates the robot's pose using a {@link SwerveDrivePoseEstimator}, and robot pose sources.
 *
 * @author Shriqui
 */
public class PoseEstimator extends SubsystemBase implements Loggable {
    private final static PoseEstimator INSTANCE = new PoseEstimator();

    private final Swerve swerve = RobotContainer.SWERVE;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    @Log
    private final Field2d field = new Field2d();
    private final List<RobotPoseSource> robotPoseSources = new ArrayList<>();

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
    }

    /**
     * @return the estimated pose of the robot
     */
    public Pose2d getCurrentPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * Adds robot pose sources to use for the pose estimator.
     *
     * @param robotPoseSources the pose sources
     */
    public void addRobotPoseSources(RobotPoseSource... robotPoseSources) {
        this.robotPoseSources.addAll(List.of(robotPoseSources));
    }

    private void updatePoseEstimator() {
        updatePoseEstimatorStates();
        attemptToUpdateWithRobotPoseSources();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithRobotPoseSources() {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            if (robotPoseSource.isNewTimestamp())
                updateWithRobotPoseSource(robotPoseSource);
        }
    }

    private void updateWithRobotPoseSource(RobotPoseSource robotPoseSource) {
        final Pose2d robotPose = robotPoseSource.getRobotPose();

        swerveDrivePoseEstimator.addVisionMeasurement(
                robotPose,
                robotPoseSource.getLastResultTimestamp()
        );

        field.getObject(robotPoseSource.getName()).setPose(robotPose);
    }

    private void updatePoseEstimatorStates() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getModulePositions());
    }

    private void addAprilTagsToFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = PoseSourceConstants.TAGS_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            field.getObject("Tag " + currentID).setPose(tagPose);
        }
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

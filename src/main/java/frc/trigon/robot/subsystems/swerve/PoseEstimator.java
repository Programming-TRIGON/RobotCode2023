package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.robotposesources.PoseSourceConstants;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.utilities.AllianceUtilities;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A class that estimates the robot's pose using a {@link SwerveDrivePoseEstimator}, and robot pose sources.
 * This pose estimator will provide you the robot's pose relative to the current driver station.
 *
 * @author Shriqui - Captain, Omer - Programing Captain
 */
public class PoseEstimator extends SubsystemBase implements Loggable {
    private final static PoseEstimator INSTANCE = new PoseEstimator();

    private final Swerve swerve = RobotContainer.SWERVE;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    @Log
    private final Field2d field = new Field2d();
    private final List<RobotPoseSource> robotPoseSources = new ArrayList<>();
    private DriverStation.Alliance lastAlliance = DriverStation.getAlliance();

    private PoseEstimator() {
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
                swerve.getHeading(),
                swerve.getModulePositions(),
                new Pose2d(),
                PoseEstimatorConstants.STATES_AMBIGUITY,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );

        putAprilTagsOnFieldWidget();
    }

    public static PoseEstimator getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        if (didAllianceChange())
            updateFieldWidget();
    }

    /**
     * @return the field widget
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to
     */
    public void resetPose(Pose2d currentPose) {
        final Pose2d currentBluePose = AllianceUtilities.toAlliancePose(currentPose);
        swerve.setHeading(currentBluePose.getRotation());

        new Notifier(() -> resetPoseEstimator(currentBluePose)).startSingle(PoseEstimatorConstants.GYRO_UPDATE_TIME_SECONDS);
    }

    /**
     * @return the estimated pose of the robot, relative to the current driver station
     */
    public Pose2d getCurrentPose() {
        return AllianceUtilities.toAlliancePose(swerveDrivePoseEstimator.getEstimatedPosition());
    }

    /**
     * Adds robot pose sources to use for the pose estimator.
     *
     * @param robotPoseSources the pose sources
     */
    public void addRobotPoseSources(RobotPoseSource... robotPoseSources) {
        this.robotPoseSources.addAll(List.of(robotPoseSources));
    }

    private void updateFieldWidget() {
        putAprilTagsOnFieldWidget();
        lastAlliance = DriverStation.getAlliance();
    }

    private boolean didAllianceChange() {
        return lastAlliance != DriverStation.getAlliance();
    }

    private void resetPoseEstimator(Pose2d currentPose) {
        swerveDrivePoseEstimator.resetPosition(
                swerve.getHeading(),
                swerve.getModulePositions(),
                currentPose
        );
    }

    private void updatePoseEstimator() {
        updatePoseEstimatorStates();
        attemptToUpdateWithRobotPoseSources();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithRobotPoseSources() {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            if (robotPoseSource.hasNewResult())
                updateFromPoseSource(robotPoseSource);
        }
    }

    private void updateFromPoseSource(RobotPoseSource robotPoseSource) {
        final Pose2d robotPose = robotPoseSource.getRobotPose();

        field.getObject(robotPoseSource.getName()).setPose(robotPose);
        swerveDrivePoseEstimator.addVisionMeasurement(
                robotPose,
                robotPoseSource.getLastResultTimestamp()
        );
    }

    private void updatePoseEstimatorStates() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getModulePositions());
    }

    private void putAprilTagsOnFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = PoseSourceConstants.TAGS_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            field.getObject("Tag " + currentID).setPose(AllianceUtilities.toAlliancePose(tagPose));
        }
    }
}

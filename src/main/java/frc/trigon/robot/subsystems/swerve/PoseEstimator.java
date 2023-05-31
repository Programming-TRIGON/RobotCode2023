package frc.trigon.robot.subsystems.swerve;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.robotposesources.PoseSourceConstants;
import frc.trigon.robot.robotposesources.RelativeRobotPoseSource;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.utilities.AllianceUtilities;
import swervelib.SwerveDrive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A class that estimates the robot's pose using a {@link SwerveDrivePoseEstimator}, and robot pose sources.
 * This pose estimator will provide you the robot's pose relative to the current driver station.
 *
 * @author Shriqui - Captain, Omer - Programing Captain
 */
public class PoseEstimator extends SubsystemBase {
    private final static PoseEstimator INSTANCE = new PoseEstimator();

    private final Swerve swerve = RobotContainer.SWERVE;
    private final SwerveDrive swerveDrive = swerve.getSwerveDrive();
    private final List<RobotPoseSource> robotPoseSources = new ArrayList<>();
    private DriverStation.Alliance lastAlliance = DriverStation.getAlliance();

    public static PoseEstimator getInstance() {
        return INSTANCE;
    }

    private PoseEstimator() {
        swerveDrive.stateStdDevs = PoseEstimatorConstants.STATES_AMBIGUITY;

        putAprilTagsOnFieldWidget();
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        if (didAllianceChange())
            updateFieldWidget();
    }

    /**
     * @return the estimated pose of the robot, relative to the current driver station
     */
    public Pose2d getCurrentPose() {
        return AllianceUtilities.toAlliancePose(swerveDrive.getPose());
    }

    /**
     * @return the field widget
     */
    public Field2d getField() {
        return swerveDrive.field;
    }

    /**
     * Adds robot pose sources to use for the pose estimator.
     *
     * @param robotPoseSources the pose sources
     */
    public void addRobotPoseSources(RobotPoseSource... robotPoseSources) {
        this.robotPoseSources.addAll(List.of(robotPoseSources));
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to
     */
    public void resetPose(Pose2d currentPose) {
        final Pose2d currentBluePose = AllianceUtilities.toAlliancePose(currentPose);

        swerve.setHeading(currentBluePose.getRotation());
        // TODO: Check if this works
        swerveDrive.resetOdometry(currentPose);

        configureRelativePoseSources(currentPose);
    }

    private void updateFieldWidget() {
        putAprilTagsOnFieldWidget();
        lastAlliance = DriverStation.getAlliance();
    }

    private boolean didAllianceChange() {
        return lastAlliance != DriverStation.getAlliance();
    }

    private void configureRelativePoseSources(Pose2d currentPose) {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            if (!(robotPoseSource instanceof RelativeRobotPoseSource))
                continue;

            final RelativeRobotPoseSource relativeRobotPoseSource = (RelativeRobotPoseSource) robotPoseSource;
            relativeRobotPoseSource.setRelativePose(currentPose);
        }
    }

    private void updatePoseEstimator() {
        swerveDrive.updateOdometry();
        attemptToUpdateWithRobotPoseSources();
        getField().setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithRobotPoseSources() {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            if (robotPoseSource.hasNewResult())
                updateFromPoseSource(robotPoseSource);
        }
    }

    private void updateFromPoseSource(RobotPoseSource robotPoseSource) {
        final Pose2d robotPose = robotPoseSource.getRobotPose();

        getField().getObject(robotPoseSource.getName()).setPose(AllianceUtilities.toAlliancePose(robotPose));
        swerveDrive.addVisionMeasurement(
                robotPose,
                robotPoseSource.getLastResultTimestamp(),
                true,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );
    }

    private void putAprilTagsOnFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = PoseSourceConstants.TAGS_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            getField().getObject("April Tag [" + currentID + "]").setPose(AllianceUtilities.toAlliancePose(tagPose));
        }
    }

}


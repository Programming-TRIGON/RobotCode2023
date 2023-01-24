package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.posesources.PoseSource;
import frc.trigon.robot.posesources.PoseSourceConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;


public class PoseEstimator extends SubsystemBase implements Loggable {
    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final PoseSource[] poseSources;

    @Log
    private final Field2d field = new Field2d();

    public PoseEstimator(PoseSource... poseSources) {
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.KINEMATICS,
                swerve.getHeading(),
                swerve.getModulePositions(),
                new Pose2d(),
                PoseEstimatorConstants.STATES_AMBIGUITY,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );

        addAprilTagsToFieldWidget();

        this.poseSources = poseSources;
    }

    /**
     * @return a command that updates the pose estimator. Runs when disabled
     */
    public Command getUpdatePoseEstimatorCommand() {
        return new RunCommand(this::updatePoseEstimator, this).ignoringDisable(true);
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to
     */
    public void resetPose(Pose2d currentPose) {
        swerve.setHeading(currentPose.getRotation());

        // Wait for gyro to update
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

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
        return swerveDrivePoseEstimator == null ? new Pose2d() : swerveDrivePoseEstimator.getEstimatedPosition();
    }

    private void updatePoseEstimator() {
        attemptToUpdateWithPoseSources();
        updatePoseEstimatorStates();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithPoseSources() {
        for (PoseSource poseSource : poseSources) {
            if (!poseSource.canUpdate()) continue;

            updateWithPoseSource(poseSource);
        }
    }

    private void updateWithPoseSource(PoseSource poseSource) {
        final Pose2d robotPose = poseSource.getRobotPose(swerve.getHeading());

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

        for (int i = 0; i < tagsCount; i++) {
            final Pose3d tagPose = tagPoses.get(i);

            field.getObject("Tag " + i).setPose(tagPose.toPose2d());
        }
    }

}

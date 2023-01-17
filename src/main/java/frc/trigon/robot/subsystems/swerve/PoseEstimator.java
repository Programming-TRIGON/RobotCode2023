package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.vision.AprilTagCamera;
import frc.trigon.robot.vision.VisionConstants;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;


public class PoseEstimator extends SubsystemBase {
    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final AprilTagCamera camera;

    @Log
    private final Field2d field = new Field2d();

    public PoseEstimator(AprilTagCamera camera) {
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.KINEMATICS,
                swerve.getHeading(),
                swerve.getModulePositions(),
                new Pose2d(),
                PoseEstimatorConstants.STATES_AMBIGUITY,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );

        addAprilTagsToField();

        this.camera = camera;
    }

    /**
     * @return a command that updates the pose estimator and runs when the robot is disabled
     */
    public Command getUpdatePoseEstimatorCommand() {
        return new RunCommand(this::updatePoseEstimator, this).ignoringDisable(true);
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param startingPose the pose to reset to
     */
    public void resetPose(Pose2d startingPose) {
        swerve.setHeading(startingPose.getRotation());

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        swerveDrivePoseEstimator.resetPosition(
                swerve.getHeading(),
                swerve.getModulePositions(),
                startingPose
        );
    }

    /**
     * @return the current pose of the robot
     */
    public Pose2d getCurrentPose() {
        if (swerveDrivePoseEstimator == null) return new Pose2d();

        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    private void updatePoseEstimator() {
        attemptToAddVisionMeasurement();
        updatePoseEstimatorStates();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToAddVisionMeasurement() {
        if (!camera.hasNewResult() || !camera.doesHaveGoodTag()) return;

        swerveDrivePoseEstimator.addVisionMeasurement(camera.getRobotPose().toPose2d(), Timer.getFPGATimestamp());
    }

    private void updatePoseEstimatorStates() {
        final SwerveModulePosition defaultModulePosition = new SwerveModulePosition(0, new Rotation2d());

        final SwerveModulePosition[] modulePositions = {
                defaultModulePosition, defaultModulePosition,
                defaultModulePosition, defaultModulePosition
        };

        swerveDrivePoseEstimator.update(new Rotation2d(), modulePositions);
    }

    private void addAprilTagsToField() {
        final List<Pose3d> tagPoses = VisionConstants.TAG_POSES;
        final int tagsCount = tagPoses.size();

        for (int i = 0; i < tagsCount; i++) {
            final Pose3d tagPose = tagPoses.get(i);

            field.getObject("Tag " + i).setPose(tagPose.toPose2d());
        }
    }

}

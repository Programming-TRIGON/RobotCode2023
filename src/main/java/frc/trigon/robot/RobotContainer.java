package frc.trigon.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.robotposesources.AprilTagPhotonCamera;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.robotposesources.T265;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.trihard.TrihardSwerve;

public class RobotContainer {
    public static final Swerve SWERVE = TrihardSwerve.getInstance();
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
    private final Transform3d
            forwardLimelightToRobotCenter = new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0)
            ),
            t265ToRobotCenter = new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0)
            );
    private final RobotPoseSource
            forwardLimelight = new AprilTagPhotonCamera("limelight-forward", forwardLimelightToRobotCenter),
            t265 = new T265("t265", t265ToRobotCenter);

    public RobotContainer() {
        setPoseEstimatorPoseSources();
    }

    private void setPoseEstimatorPoseSources() {
        poseEstimator.setRobotPoseSources(forwardLimelight, t265);
    }
}

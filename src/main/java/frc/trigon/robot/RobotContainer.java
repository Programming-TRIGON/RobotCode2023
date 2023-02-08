package frc.trigon.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.posesources.PhotonCamera;
import frc.trigon.robot.posesources.PoseSource;
import frc.trigon.robot.subsystems.swerve.PoseEstimator;

public class RobotContainer {
    private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
    private final Transform3d photonCameraToRobotCenter = new Transform3d(
            new Translation3d(0 ,0, 0),
            new Rotation3d(0, 0, 0)
    );
    private final PoseSource photonCamera = new PhotonCamera("limelime", photonCameraToRobotCenter);

    public RobotContainer() {
        poseEstimator.setPoseSources(photonCamera);
    }
}
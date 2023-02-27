package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.robotposesources.AprilTagPhotonCamera;
import frc.trigon.robot.robotposesources.RobotPoseSource;

public class CameraConstants {
    private static final Transform3d FORWARD_LIMELIGHT_TO_ROBOT = new Transform3d(
            new Translation3d(-0.0355, 0.2, -1.04),
            new Rotation3d(0, Math.toRadians(-20.9), Math.toRadians(7.6346))
    );
//    private static final Transform3d FORWARD_LIMELIGHT_TO_ROBOT = new Transform3d(
//            new Translation3d(-0.0355, 0.2, -1.04),
//            new Rotation3d(0, Math.toRadians(-20.9), Math.toRadians(-7.6346))
//    );
    public static final RobotPoseSource FORWARD_LIMELIGHT = new AprilTagPhotonCamera(
            "limelight-forward",
            FORWARD_LIMELIGHT_TO_ROBOT
    );
}

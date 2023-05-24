package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.robotposesources.AprilTagPhotonCamera;
import frc.trigon.robot.robotposesources.RobotPoseSource;

public class CameraConstants {
    public static final Transform3d FORWARD_LIMELIGHT_TO_ROBOT = new Transform3d(
            new Pose3d(
                    new Translation3d(
                            1.778320, 4.655098, 1.180738
                    ),
                    new Rotation3d(
                            -0.073926, 0.508936, 2.901368
                    )
            ),
            new Pose3d(
                    new Translation3d(
                            1.834936, 4.424439, 0
                    ),
                    new Rotation3d(
                            0, 0, Math.toRadians(180)
                    )
            )
    );
    public static final RobotPoseSource FORWARD_LIMELIGHT = new AprilTagPhotonCamera(
            "limelight-forward",
            FORWARD_LIMELIGHT_TO_ROBOT
    );
//    public static final T265 t265 = new T265("908412110743", new Transform3d());

}

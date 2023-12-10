package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.robotposesources.PoseSourceConstants;
import frc.trigon.robot.robotposesources.RelativeRobotPoseSource;
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
    public static final RobotPoseSource
            FORWARD_LIMELIGHT = new RobotPoseSource(
                    PoseSourceConstants.RobotPoseSourceType.PHOTON_CAMERA,
                    "limelight-forward",
                    FORWARD_LIMELIGHT_TO_ROBOT
            );
    public static final RelativeRobotPoseSource T265 = new RelativeRobotPoseSource(
                    PoseSourceConstants.RobotPoseSourceType.T265,
                    "908412110743",
                    new Transform2d(new Translation2d(-0.1, 0), Rotation2d.fromDegrees(180))
            );

}

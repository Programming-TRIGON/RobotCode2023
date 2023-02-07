package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.posesources.PhotonCamera;
import frc.trigon.robot.posesources.PoseSource;

import java.util.List;

public class PoseEstimatorConstants {
    private static final Transform3d PHOTON_CAMERA_TO_ROBOT_CENTER = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );
    static final int GYRO_UPDATE_DELAY_MS = 15;
    static final List<PoseSource> POSE_SOURCES = List.of(
            new PhotonCamera("limelime", PHOTON_CAMERA_TO_ROBOT_CENTER)
    );

    /**
     * The vector represents how ambiguous is each value.
     * <p>
     * The first value represents how ambiguous is the x,
     * the second one for the y, and the third one is for the theta (rotation).
     * <p>
     * Increase these numbers to trust the estimate less.
     */
    static final Vector<N3>
            STATES_AMBIGUITY = VecBuilder.fill(0.005, 0.005, 0.0005),
            VISION_CALCULATIONS_AMBIGUITY = VecBuilder.fill(0.05, 0.05, 0.005);
}

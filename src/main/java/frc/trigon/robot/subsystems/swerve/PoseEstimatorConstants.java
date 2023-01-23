package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class PoseEstimatorConstants {
    /**
     * The vector represents how much the pose estimator can trust the process in each value.
     * <p>
     * The first value represents how much it can trust for the x,
     * the second one for the y, and the third one for the theta (rotation).
     */
    static final Vector<N3>
            STATES_AMBIGUITY = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0.01)),
            VISION_CALCULATIONS_AMBIGUITY = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(4));
}

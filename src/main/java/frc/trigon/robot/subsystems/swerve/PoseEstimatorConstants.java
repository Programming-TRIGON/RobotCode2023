package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class PoseEstimatorConstants {
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
            VISION_CALCULATIONS_AMBIGUITY = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(4));
}

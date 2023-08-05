package frc.trigon.robot.subsystems.poseestimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;

public class PoseEstimatorConstants {
    private static final PoseLimiter.StraightLine2d[] FIELD_LIMIT_LINES = {
            new PoseLimiter.StraightLine2d(new Translation2d(1.37778, 0), new Translation2d(15.16293, 0)),
            new PoseLimiter.StraightLine2d(new Translation2d(1.37778, 0), new Translation2d(1.37778, 5.48711)),
            new PoseLimiter.StraightLine2d(new Translation2d(1.37778, 5.48711), new Translation2d(3.35899, 5.48711)),
            new PoseLimiter.StraightLine2d(new Translation2d(3.35899, 5.48711), new Translation2d(3.35899, 5.48711 + 0.02153)),
            new PoseLimiter.StraightLine2d(new Translation2d(0.34608, 5.48711 + 0.02153), new Translation2d(3.35899, 5.48711 + 0.02153)),
            new PoseLimiter.StraightLine2d(new Translation2d(0.34608, 5.48711 + 0.02153), new Translation2d(0.34608, 8.01668)),
            new PoseLimiter.StraightLine2d(new Translation2d(0.34608, 8.01668), new Translation2d(16.18528, 8.01668)),
            new PoseLimiter.StraightLine2d(new Translation2d(16.18528, 5.48711 + 0.02153), new Translation2d(16.18528, 8.01668)),
            new PoseLimiter.StraightLine2d(new Translation2d(13.18173, 5.48711 + 0.02153), new Translation2d(16.18528, 5.48711 + 0.02153)),
            new PoseLimiter.StraightLine2d(new Translation2d(13.18173, 5.48711), new Translation2d(13.18173, 5.48711 + 0.02153)),
            new PoseLimiter.StraightLine2d(new Translation2d(13.18173, 5.48711), new Translation2d(15.16293, 5.48711)),
            new PoseLimiter.StraightLine2d(new Translation2d(15.16293, 0), new Translation2d(15.16293, 5.48711)),
    };
    private static final double BUMPERS_LENGTH = 0.08;

    static final double GYRO_UPDATE_TIME_SECONDS = 0.020;
    static final boolean LIMIT_POSITION = ConfigurationConstants.ROBOT_TYPE == ConfigurationConstants.RobotType.SIMULATION;

    static final PoseLimiter POSE_LIMITER = new PoseLimiter(Swerve.getInstance().getConstants().getRobotSideLength() + BUMPERS_LENGTH, FIELD_LIMIT_LINES);

    /**
     * The vector represents how ambiguous each value is.
     * The first value represents how ambiguous is the x,
     * the second one for the y, and the third one is for the theta (rotation).
     * Increase these numbers to trust the estimate less.
     */
    static final Vector<N3>
            STATES_AMBIGUITY = VecBuilder.fill(0.005, 0.005, 555555555), // 0.00005
            VISION_CALCULATIONS_AMBIGUITY = VecBuilder.fill(0.1, 0.05, Math.toRadians(720));
}


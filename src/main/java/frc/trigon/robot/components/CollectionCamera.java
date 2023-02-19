package frc.trigon.robot.components;

import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.utilities.Maths;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A class that represents a collection camera that looks at the collection and detects game pieces.
 */
public class CollectionCamera extends PhotonCamera {
    private static final int
            CONES_DETECTION_PIPELINE_INDEX = 0,
            CUBES_DETECTION_PIPELINE_INDEX = 1;
    private static final double
            YAW_TO_POSITION_POLYNOMIAL_A = 1,
            YAW_TO_POSITION_POLYNOMIAL_B = 1,
            YAW_TO_POSITION_POLYNOMIAL_C = 1;
    private static final double NOTIFIER_PERIOD_SECONDS = 1;
    private PhotonTrackedTarget
            lastBestCone = null,
            lastBestCube = null;

    /**
     * Constructs a new collection camera.
     *
     * @param hostname the name of the collection camera
     */
    public CollectionCamera(String hostname) {
        super(hostname);

        final Notifier updateTargetsNotifier = new Notifier(this::updatePipelines);
        updateTargetsNotifier.startPeriodic(NOTIFIER_PERIOD_SECONDS);
    }

    /**
     * @return the position of the game piece on the collection, in meters. Returns the default position if no game piece is detected
     */
    public double getPositionOnCollection(double defaultPosition) {
        final PhotonTrackedTarget bestTargetGamePiece = getTargetGamePiece();
        if (bestTargetGamePiece == null)
            return defaultPosition;

        final double gamePieceYaw = bestTargetGamePiece.getYaw();

        return calculatePositionOnCollection(gamePieceYaw);
    }

    /**
     * @return the type of game piece that the collection limelight sees
     */
    public GamePieceType getTargetGamePieceType() {
        if (getTargetGamePiece() == null)
            return GamePieceType.NONE;
        if (lastBestCube == null)
            return GamePieceType.CONE;
        if (lastBestCone == null)
            return GamePieceType.CUBE;

        return GamePieceType.NONE;
    }

    private PhotonTrackedTarget getTargetGamePiece() {
        if (lastBestCube == null)
            return lastBestCone;
        if (lastBestCone == null)
            return lastBestCube;

        return null;
    }

    private void updatePipelines() {
        if (getPipelineIndex() == CONES_DETECTION_PIPELINE_INDEX) {
            lastBestCone = getLatestResult().getBestTarget();
            setPipelineIndex(CUBES_DETECTION_PIPELINE_INDEX);
        } else {
            lastBestCube = getLatestResult().getBestTarget();
            setPipelineIndex(CONES_DETECTION_PIPELINE_INDEX);
        }
    }

    private double calculatePositionOnCollection(double gamePieceYaw) {
        return Maths.calculatePolynomial(
                gamePieceYaw,
                YAW_TO_POSITION_POLYNOMIAL_A,
                YAW_TO_POSITION_POLYNOMIAL_B,
                YAW_TO_POSITION_POLYNOMIAL_C
        );
    }

    public enum GamePieceType {
        CONE,
        CUBE,
        NONE
    }
}

package frc.trigon.robot.components;

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
            A = 1,
            B = 1,
            C = 1;

    /**
     * Constructs a new collection camera.
     *
     * @param hostname the name of the collection camera
     */
    public CollectionCamera(String hostname) {
        super(hostname);
    }

    /**
     * @return the pose of the game piece on the collection, in meters. Return the default if no game piece is detected
     */
    public double getPositionOnCollection(double defaultPosition) {
        final PhotonTrackedTarget bestTargetGamePiece = getBestTargetGamePiece();
        if (bestTargetGamePiece == null)
            return defaultPosition;

        final double gamePieceYaw = bestTargetGamePiece.getYaw();

        return calculatePositionOnCollection(gamePieceYaw);
    }

    /**
     * @return the type of game piece that the collection limelight sees
     */
    public GamePieceType getBestVisibleGamePiece() {
        final PhotonTrackedTarget targetCone = getBestTargetFromPipeline(CONES_DETECTION_PIPELINE_INDEX);
        final PhotonTrackedTarget bestTarget = getBestTargetGamePiece();

        if (bestTarget == null)
            return GamePieceType.NONE;

        final double
                coneAmbiguity = targetCone.getPoseAmbiguity(),
                bestTargetAmbiguity = bestTarget.getPoseAmbiguity();

        if (coneAmbiguity == bestTargetAmbiguity)
            return GamePieceType.CONE;
        return GamePieceType.CUBE;
    }

    private PhotonTrackedTarget getBestTargetGamePiece() {
        final PhotonTrackedTarget targetCone = getBestTargetFromPipeline(CONES_DETECTION_PIPELINE_INDEX);
        final PhotonTrackedTarget targetCube = getBestTargetFromPipeline(CUBES_DETECTION_PIPELINE_INDEX);

        if (targetCone == null)
            return targetCube;
        if (targetCube == null)
            return targetCone;

        final double coneAmbiguity = targetCone.getPoseAmbiguity();
        final double cubeAmbiguity = targetCube.getPoseAmbiguity();

        return coneAmbiguity < cubeAmbiguity || cubeAmbiguity == -1 ? targetCone : targetCube;
    }

    private PhotonTrackedTarget getBestTargetFromPipeline(int pipelineIndex) {
        setPipelineIndex(pipelineIndex);

        return getLatestResult().getBestTarget();
    }

    private double calculatePositionOnCollection(double gamePieceYaw) {
        return Maths.calculatePolynomial(gamePieceYaw, A, B, C);
    }

    public enum GamePieceType {
        CONE,
        CUBE,
        NONE
    }

}

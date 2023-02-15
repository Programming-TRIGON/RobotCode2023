package frc.trigon.robot.components;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.utilities.Maths;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CollectionLimelight extends PhotonCamera {
    private static final int
            CONES_DETECTION_PIPELINE_INDEX = 0,
            CUBES_DETECTION_PIPELINE_INDEX = 1;
    private static final double COLLECTION_LENGTH_METERS = 0.4;
    private static final double
            A = 1,
            B = 1,
            C = 1;

    /**
     * Constructs a new collection limelight.
     *
     * @param hostname the name of the collection limelight
     */
    public CollectionLimelight(String hostname) {
        super(hostname);
    }

    /**
     * @return the pose of the game piece on the collection, in meters. Return the default if no game piece is detected
     */
    public double getPoseOnCollection(double defaultPose) {
        final PhotonTrackedTarget bestTarget = getBestTarget();

        if (bestTarget == null)
            return defaultPose;

        final Pose3d gamePiecePose = transform3dToPose3d(bestTarget.getBestCameraToTarget());
        final double distanceToGamePiece = getDistance(gamePiecePose, new Pose3d());
        final double gamePieceXPixels = bestTarget.getPitch();
        final double collectionLengthPixels = getCollectionLengthPixels(distanceToGamePiece);
        final double pixelsPerMeter = collectionLengthPixels / COLLECTION_LENGTH_METERS;

        return gamePieceXPixels / pixelsPerMeter;
    }

    /**
     * @return the type of game piece that the collection limelight sees
     */
    public GamePieceType getBestVisibleGamePiece() {
        final PhotonTrackedTarget targetCone = getBestTargetFromPipeline(CONES_DETECTION_PIPELINE_INDEX);
        final PhotonTrackedTarget bestTarget = getBestTarget();

        if (bestTarget == null)
            return GamePieceType.NONE;

        final double
                coneAmbiguity = targetCone.getPoseAmbiguity(),
                bestTargetAmbiguity = bestTarget.getPoseAmbiguity();

        if (coneAmbiguity == bestTargetAmbiguity)
            return GamePieceType.CONE;
        return GamePieceType.CUBE;
    }

    private double getDistance(Pose3d pose, Pose3d other) {
        return pose.getTranslation().getDistance(other.getTranslation());
    }

    private Pose3d transform3dToPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    private PhotonTrackedTarget getBestTarget() {
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

    private double getCollectionLengthPixels(double distanceToGamePiece) {
        return Maths.calculatePolynomial(distanceToGamePiece, A, B, C);
    }

    public enum GamePieceType {
        CONE,
        CUBE,
        NONE
    }

}

package frc.trigon.robot.components.cameras.collectioncamera;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCollectionCameraIO extends CollectionCameraIO {
    private static final int
            CONES_DETECTION_PIPELINE_INDEX = 0,
            CUBES_DETECTION_PIPELINE_INDEX = 1;
    private final PhotonCamera photonCamera;
    private PhotonTrackedTarget
            lastBestCube = null,
            lastBestCone = null;

    /**
     * Constructs a new collection camera.
     *
     * @param hostname the name of the collection camera
     */
    protected PhotonCollectionCameraIO(String hostname) {
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(CollectionCameraInputsAutoLogged inputs) {
        inputs.hasTargets = photonCamera.getLatestResult().hasTargets();
        inputs.isViewingCone = isViewingCone();
        inputs.bestGamePieceYaw = getBestGamePieceYaw();
    }

    protected void updatePipelines() {
        if (isConesDetectionPipeline()) {
            updateConesPipeline();
            photonCamera.setPipelineIndex(CUBES_DETECTION_PIPELINE_INDEX);
        } else {
            updateCubesPipeline();
            photonCamera.setPipelineIndex(CONES_DETECTION_PIPELINE_INDEX);
        }
    }

    private boolean isViewingCone() {
        return lastBestCone != null;
    }

    private double getBestGamePieceYaw() {
        if (lastBestCube != null)
            return lastBestCube.getYaw();
        if (lastBestCone != null)
            return lastBestCone.getYaw();
        return 0;
    }

    private boolean isConesDetectionPipeline() {
        return photonCamera.getPipelineIndex() == CONES_DETECTION_PIPELINE_INDEX;
    }

    private void updateConesPipeline() {
        if (photonCamera.getLatestResult().hasTargets())
            lastBestCone = getTargetGamePiece();
        else
            lastBestCone = null;
    }

    private void updateCubesPipeline() {
        if (photonCamera.getLatestResult().hasTargets())
            lastBestCube = getTargetGamePiece();
        else
            lastBestCube = null;
    }

    private PhotonTrackedTarget getTargetGamePiece() {
        return photonCamera.getLatestResult().getBestTarget();
    }
}

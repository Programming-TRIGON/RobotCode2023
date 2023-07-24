package frc.trigon.robot.components.cameras.collectioncamera;

import org.littletonrobotics.junction.AutoLog;

public class CollectionCameraIO {
    protected CollectionCameraIO() {
    }

    @AutoLog
    public static class CollectionCameraInputs {
        public boolean hasTargets = false;
        public boolean isViewingCone = false;
        public double bestGamePieceYaw = 0;
    }

    /**
     * Updates the inputs of the collection camera.
     *
     * @param inputs the inputs to update
     */
    protected void updateInputs(CollectionCameraInputsAutoLogged inputs) {
    }

    /**
     * Updates the pipelines of the collection camera.
     */
    protected void updatePipelines() {
    }
}

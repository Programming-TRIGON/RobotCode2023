package frc.trigon.robot.components;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.utilities.Maths;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A class that represents a collection camera that looks at the collection system and detects game pieces.
 */
public class CollectionCamera extends PhotonCamera implements Loggable, Sendable {
    private static final int
            CONES_DETECTION_PIPELINE_INDEX = 0,
            CUBES_DETECTION_PIPELINE_INDEX = 0;
    private static final double
            YAW_TO_POSITION_POLYNOMIAL_A = 0,
            YAW_TO_POSITION_POLYNOMIAL_B = 0.98306,
            YAW_TO_POSITION_POLYNOMIAL_C = -3.98191;
    private static final double PIPELINES_SWITCHING_RATE = 99999;
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
        this.setVersionCheckEnabled(false);

        new Notifier(this::updatePipelines).startPeriodic(PIPELINES_SWITCHING_RATE);
    }

    /**
     * @return the position of the target game piece on the collection system, in meters. Returns the default position if no game piece is detected
     */
    @Log(name = "Game Piece Position")
    public double getGamePiecePosition() {
        final PhotonTrackedTarget targetGamePiece = getTargetGamePiece();
        if(targetGamePiece == null)
            return 0;

        return calculateGamePiecePosition(targetGamePiece.getYaw());
    }

    /**
     * @return the type of game piece that the collection limelight sees
     */
    @Log(name = "Game Piece Type", methodName = "name")
    public GamePieceType getTargetGamePieceType() {
        if (lastBestCube != null)
            return GamePieceType.CUBE;
        if (lastBestCone != null)
            return GamePieceType.CONE;

        return GamePieceType.NONE;
    }

    private PhotonTrackedTarget getTargetGamePiece() {
        return getLatestResult().getBestTarget();
    }

    private void updatePipelines() {
        if (getPipelineIndex() == CONES_DETECTION_PIPELINE_INDEX) {
            lastBestCone = getTargetGamePiece();
            if(lastBestCone != null)
                lastBestCube = null;
            setPipelineIndex(CUBES_DETECTION_PIPELINE_INDEX);
        } else {
            lastBestCube = getTargetGamePiece();
            if(lastBestCube != null)
                lastBestCone = null;
            setPipelineIndex(CONES_DETECTION_PIPELINE_INDEX);
        }
    }

    private double calculateGamePiecePosition(double gamePieceYaw) {
        return Maths.calculatePolynomial(
                YAW_TO_POSITION_POLYNOMIAL_A,
                YAW_TO_POSITION_POLYNOMIAL_B,
                YAW_TO_POSITION_POLYNOMIAL_C,
                gamePieceYaw
        );
    }

    public enum GamePieceType {
        CONE,
        CUBE,
        NONE
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("pos", this::getGamePiecePosition, null);
    }
}

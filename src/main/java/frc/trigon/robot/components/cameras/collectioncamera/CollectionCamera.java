package frc.trigon.robot.components.cameras.collectioncamera;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;
import frc.trigon.robot.utilities.Maths;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

/**
 * A class that represents a collection camera that looks at the collection system and detects game pieces.
 */
public class CollectionCamera extends SubsystemBase {
    private static final double
            YAW_TO_POSITION_POLYNOMIAL_A = 0,
            YAW_TO_POSITION_POLYNOMIAL_B = 0.98306,
            YAW_TO_POSITION_POLYNOMIAL_C = -3.98191;
    private static final double PIPELINES_SWITCHING_RATE = 99999;
    private final CollectionCameraInputsAutoLogged collectionCameraInputs = new CollectionCameraInputsAutoLogged();
    private final CollectionCameraIO collectionCameraIO;
    private final String hostname;

    /**
     * Constructs a new collection camera.
     *
     * @param hostname the name of the collection camera
     */
    public CollectionCamera(String hostname) {
        this.hostname = hostname;
        collectionCameraIO = generateIO();

        new Notifier(collectionCameraIO::updatePipelines).startPeriodic(PIPELINES_SWITCHING_RATE);
        PhotonCamera.setVersionCheckEnabled(false);
    }

    @Override
    public void periodic() {
        collectionCameraIO.updateInputs(collectionCameraInputs);
        Logger.getInstance().processInputs(hostname, collectionCameraInputs);
        Logger.getInstance().recordOutput(hostname + "/gamePiecePosition", getGamePiecePosition());
    }

    /**
     * @return the type of game piece that the collection limelight sees
     */
    public GamePieceType getTargetGamePieceType() {
        if (collectionCameraInputs.isViewingCone)
            return GamePieceType.CONE;
        if (collectionCameraInputs.hasTargets)
            return GamePieceType.CUBE;
        return GamePieceType.NONE;
    }

    /**
     * @return the position of the target game piece on the collection system, in meters. Returns the default position if no game piece is detected
     */
    public double getGamePiecePosition() {
        if (!collectionCameraInputs.hasTargets)
            return 0;
        return calculateGamePiecePosition(collectionCameraInputs.bestGamePieceYaw);
    }

    private double calculateGamePiecePosition(double gamePieceYaw) {
        return Maths.calculatePolynomial(
                YAW_TO_POSITION_POLYNOMIAL_A,
                YAW_TO_POSITION_POLYNOMIAL_B,
                YAW_TO_POSITION_POLYNOMIAL_C,
                gamePieceYaw
        );
    }

    private CollectionCameraIO generateIO() {
        if (!Robot.IS_REAL)
            return new CollectionCameraIO();

        return new PhotonCollectionCameraIO(hostname);
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

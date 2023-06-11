package frc.trigon.robot.components;

import com.google.gson.annotations.SerializedName;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.utilities.JsonHandler;

import java.util.ArrayList;
import java.util.List;

public class D415 {
    private static final NetworkTable NETWORK_TABLE = NetworkTableInstance.getDefault().getTable("D415");
    private final NetworkTableEntry jsonDump;

    public D415(String name) {
        jsonDump = NETWORK_TABLE.getEntry(name + "/jsonDump");
    }

    /**
     * @return a list of the visible game pieces
     */
    public List<GamePiece> getVisibleGamePieces() {
        final List<GamePiece> gamePieces = new ArrayList<>();
        final D415JsonDump jsonDump = getJsonDump();

        for (D415JsonDump.Result currentResult : jsonDump.results) {
            gamePieces.add(new GamePiece(
                    isCone(currentResult),
                    getPose(currentResult)
            ));
        }

        return gamePieces;
    }

    /**
     * @return true if there are visible game pieces, false otherwise
     */
    public boolean hasVisibleGamePieces() {
        return getJsonDump().results.length > 0;
    }

    /**
     * @return the best target game piece, or null if none are visible
     */
    public GamePiece getBestGamePiece() {
        GamePiece bestGamePiece = null;
        double lastConfidence = 0;

        for (D415JsonDump.Result currentResult : getJsonDump().results) {
            if (currentResult.confidence > lastConfidence) {
                bestGamePiece = new GamePiece(
                        isCone(currentResult),
                        getPose(currentResult)
                );
                lastConfidence = currentResult.confidence;
            }
        }

        return bestGamePiece;
    }

    private boolean isCone(D415JsonDump.Result result) {
        return result.classId == 0;
    }

    private Pose3d getPose(D415JsonDump.Result result) {
        final Translation3d translation = new Translation3d(
                result.translation[0],
                result.translation[1],
                result.translation[2]
        );
        final Rotation3d rotation = new Rotation3d(new Quaternion(
                result.rotation[0],
                result.rotation[1],
                result.rotation[2],
                result.rotation[3]
        ));

        return new Pose3d(translation, rotation);
    }

    private D415JsonDump getJsonDump() {
        return JsonHandler.parseJsonStringToObject(jsonDump.getString(""), D415JsonDump.class);
    }

    public static class GamePiece {
        private final boolean isCone;
        private final Pose3d pose;

        GamePiece(boolean isCone, Pose3d pose) {
            this.isCone = isCone;
            this.pose = pose;
        }

        public boolean isCone() {
            return isCone;
        }

        public Pose3d getPose() {
            return pose;
        }
    }

    private static class D415JsonDump {
        private Result[] results;

        private static class Result {
            private double[] translation;
            private double[] rotation;
            private double confidence;
            @SerializedName("class")
            private int classId;
        }
    }

}
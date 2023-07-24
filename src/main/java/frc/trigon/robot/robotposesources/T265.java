package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.utilities.JsonHandler;

public class T265 extends RobotPoseSourceIO {
    private static final NetworkTable NETWORK_TABLE = NetworkTableInstance.getDefault().getTable("T265");
    private final NetworkTableEntry jsonDump;
    private static final short CONFIDENCE_THRESHOLD = 2;

    protected T265(String name) {
        jsonDump = NETWORK_TABLE.getEntry(name + "/jsonDump");
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = canUseJsonDump();
        if (inputs.hasResult)
            inputs.cameraPose = RobotPoseSource.pose3dToDoubleArray(getCameraPose());
        inputs.lastResultTimestamp = jsonDump.getLastChange();
    }

    private Pose3d getCameraPose() {
        if(!canUseJsonDump())
            return new Pose3d();

        return getRobotPoseFromJsonDump();
    }

    private Pose3d getRobotPoseFromJsonDump() {
        final T265JsonDump jsonDump = getJsonDump();
        final Translation3d translation = getTranslationFromDoubleArray(jsonDump.translation);
        final Rotation3d rotation = getRotationFromDoubleArray(jsonDump.rotation);
        var eus = new CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.U(), CoordinateAxis.S());

        var converted = CoordinateSystem.convert(new Pose3d(translation, rotation), eus, CoordinateSystem.NWU());
        return new Pose3d(converted.getTranslation(), converted.getRotation().plus(new Rotation3d(0, 0, Math.toRadians(90))));
    }

    private Translation3d getTranslationFromDoubleArray(double[] xyz) {
        return new Translation3d(xyz[0], xyz[1], xyz[2]);
    }

    private Rotation3d getRotationFromDoubleArray(double[] wxyz) {
        return new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
    }

    private boolean canUseJsonDump() {
        final T265JsonDump jsonDump = getJsonDump();

        try {
            return jsonDump.confidence >= CONFIDENCE_THRESHOLD && jsonDump.translation.length == 3 && jsonDump.rotation.length == 4;
        } catch(NullPointerException e) {
            return false;
        }
    }

    private T265JsonDump getJsonDump() {
        return JsonHandler.parseJsonStringToObject(jsonDump.getString(""), T265JsonDump.class);
    }

    private static class T265JsonDump {
        private double[] translation;
        private double[] rotation;
        private int confidence;
    }
}
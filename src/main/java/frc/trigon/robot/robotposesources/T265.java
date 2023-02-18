package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.utilities.JsonHandler;

public class T265 extends RelativeRobotPoseSource {
    private static final NetworkTable NETWORK_TABLE = NetworkTableInstance.getDefault().getTable("T265");
    private final String name;
    private final NetworkTableEntry jsonDump;

    public T265(String name, Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
        this.name = name;

        jsonDump = NETWORK_TABLE.getEntry(name + "/jsonDump");
    }

    @Override
    public Pose3d getCameraPose() {
        if (!canUseJsonDump())
            return getLastProvidedPose();

        return getRobotPoseFromJsonDump();
    }

    @Override
    public double getLastResultTimestamp() {
        return jsonDump.getLastChange();
    }

    @Override
    public String getName() {
        return name;
    }

    private Pose3d getRobotPoseFromJsonDump() {
        final T265JsonDump jsonDump = getJsonDump();
        final Translation3d translation = getTranslationFromYXZArray(jsonDump.translation);
        final Rotation3d rotation = getRotationFromWXYZArray(jsonDump.rotation);

        return new Pose3d(translation, rotation);
    }

    private Translation3d getTranslationFromYXZArray(double[] yxz) {
        return new Translation3d(yxz[1], yxz[0], yxz[2]);
    }

    private Rotation3d getRotationFromWXYZArray(double[] wxyz) {
        return new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
    }

    private boolean canUseJsonDump() {
        final T265JsonDump jsonDump = getJsonDump();

        try {
            return jsonDump.confidence == 3 && jsonDump.translation.length == 3 && jsonDump.rotation.length == 4;
        } catch (NullPointerException e) {
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

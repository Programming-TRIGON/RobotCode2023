package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.utilities.JsonHandler;

public class T265 implements PoseSource {
    private final String name;
    private final NetworkTableEntry jsonDump;
    private double lastUpdatedTimestamp = 0;
    private Pose2d lastRealPose = new Pose2d();
    private Transform2d currentPoseToOffsettedPose = new Transform2d();

    public T265(String name) {
        this.name = name;
        final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("T265");

        jsonDump = networkTable.getEntry(name + "/jsonDump");
    }

    @Override
    public void setCurrentPose(Pose2d pose) {
        currentPoseToOffsettedPose = pose2dToTransform2d(pose);
    }

    @Override
    public Pose2d getLastRealPose() {
        return lastRealPose;
    }

    @Override
    public boolean hasResults() {
        return getJsonDump() != null;
    }

    @Override
    public Pose2d getRobotPose() {
        if (!canUseJsonDump())
            return null;

        final Pose2d robotWPIPose = t265PoseToWPIPose(getRobotPoseFromJsonDump().toPose2d());
        lastRealPose = robotWPIPose.plus(currentPoseToOffsettedPose);

        return lastRealPose;
    }

    @Override
    public double getTimestampSeconds() {
        return jsonDump.getLastChange();
    }

    @Override
    public double getLastUpdatedTimestamp() {
        return lastUpdatedTimestamp;
    }

    @Override
    public void setLastUpdatedTimestamp(double timestamp) {
        lastUpdatedTimestamp = timestamp;
    }

    @Override
    public String getName() {
        return name;
    }

    private Pose2d t265PoseToWPIPose(Pose2d pose) {
        return new Pose2d(pose.getTranslation().getY(), pose.getTranslation().getX(), pose.getRotation());
    }

    private Transform2d pose2dToTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    private Pose3d getRobotPoseFromJsonDump() {
        final T265JsonDump jsonDump = getJsonDump();

        final Translation3d translation = new Translation3d(
                jsonDump.translation[0],
                jsonDump.translation[1],
                jsonDump.translation[2]
        );
        final Rotation3d rotation = new Rotation3d(new Quaternion(
                jsonDump.rotation[0],
                jsonDump.rotation[1],
                jsonDump.rotation[2],
                jsonDump.rotation[3]
        ));

        return new Pose3d(translation, rotation);
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

package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.utilities.JsonHandler;

public class T265 extends RelativePoseSource {
    private static final NetworkTable NETWORK_TABLE = NetworkTableInstance.getDefault().getTable("T265");
    private final String name;
    private final Transform3d t265ToRobotCenter;
    private final NetworkTableEntry jsonDump;

    public T265(String name, Transform3d t265ToRobotCenter) {
        this.name = name;
        this.t265ToRobotCenter = t265ToRobotCenter;

        jsonDump = NETWORK_TABLE.getEntry(name + "/jsonDump");
    }

    @Override
    public Pose2d getRobotPose() {
        if (!canUseJsonDump())
            return getLastRealPose();

        setLastRealPose(t265PoseToRobotPose(getRobotPoseFromJsonDump()));
        return getLastRealPose();
    }

    @Override
    public double getLastResultTimestamp() {
        return jsonDump.getLastChange();
    }

    @Override
    public String getName() {
        return name;
    }

    private Pose2d t265PoseToRobotPose(Pose3d pose) {
        final Pose3d
                robotWpiPose = t265PoseToWpiPose(pose),
                robotCenterWpiPose = robotWpiPose.plus(t265ToRobotCenter);
        final Transform2d poseToRelativePose = getPoseToRelativePoseTransform();

        return robotCenterWpiPose.toPose2d().plus(poseToRelativePose);
    }

    private Pose3d t265PoseToWpiPose(Pose3d pose) {
        final Translation3d translation = new Translation3d(
                pose.getY(),
                pose.getX(),
                pose.getZ()
        );

        return new Pose3d(translation, pose.getRotation());
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

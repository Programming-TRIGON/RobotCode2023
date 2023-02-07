package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.utilities.JsonHandler;

@SuppressWarnings("unused")
public class Limelight implements PoseSource {
    private final String hostname;
    private final NetworkTableEntry tv, json, ledMode, driverCam, pipeline, snapshot;
    private double lastUpdatedTimestamp = 0;
    private Pose2d lastRealPose = new Pose2d();

    /**
     * Constructs a new Limelight.
     *
     * @param hostname the name of the Limelight
     */
    public Limelight(String hostname) {
        this.hostname = hostname;
        final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(hostname);

        tv = networkTable.getEntry("tv");
        json = networkTable.getEntry("json");
        ledMode = networkTable.getEntry("ledMode");
        driverCam = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");
        snapshot = networkTable.getEntry("snapshot");
    }

    @Override
    public Pose2d getLastRealPose() {
        return lastRealPose;
    }

    @Override
    public boolean hasResults() {
        return tv.getDouble(0) > 0;
    }

    @Override
    public Pose2d getRobotPose() {
        final Pose3d robotPose = getRobotPoseFromJsonDump();
        if (robotPose == null)
            return null;

        lastRealPose = robotPose.toPose2d();
        return robotPose.toPose2d();
    }

    @Override
    public double getTimestampSeconds() {
        return getJsonOutput().Results.ts;
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
        return hostname;
    }

    /**
     * Gets the ty value of the target with the given id, from the json dump.
     *
     * @param id the target april tag's id
     * @return the vertical offset from the crosshair to the target (-20.5 degrees to 20.5 degrees)
     */
    public double getTy(int id) {
        final LimelightJsonOutput.Results.Fiducial fiducial = getJsonOutput().Results.getFiducialFromId(id);
        if (fiducial == null)
            return 0;

        return fiducial.ty;
    }

    /**
     * Gets the tx value of the target with the given id, from the json dump.
     *
     * @param id the target april tag's id
     * @return the horizontal offset from the crosshair to the target (-27 degrees to 27 degrees)
     */
    public double getTx(int id) {
        final LimelightJsonOutput.Results.Fiducial fiducial = getJsonOutput().Results.getFiducialFromId(id);
        if (fiducial == null)
            return 0;

        return fiducial.tx;
    }

    /**
     * Gets the ta value of the target with the given id, from the json dump.
     *
     * @param id the target april tag's id
     * @return target's area (from 0% of the image to 100% of the image)
     */
    public double getTa(int id) {
        final LimelightJsonOutput.Results.Fiducial fiducial = getJsonOutput().Results.getFiducialFromId(id);
        if (fiducial == null)
            return 0;

        return fiducial.ta;
    }

    /**
     * @return true if the driver cam is used, false if the vision cam is used
     */
    public boolean isDriverCam() {
        return driverCam.getDouble(0) == 1;
    }

    /**
     * Sets the camera mode.
     *
     * @param useDriverCam true for driver camera, false for vision processing
     */
    public void setCam(boolean useDriverCam) {
        driverCam.setNumber(useDriverCam ? 1 : 0);
    }

    /**
     * @return the current LedMode
     */
    public LedMode getLedMode() {
        return LedMode.getLedModeFromValue((int) ledMode.getDouble(0));
    }

    /**
     * Sets the led mode.
     *
     * @param mode the wanted LedMode
     */
    public void setLedMode(LedMode mode) {
        ledMode.setNumber(mode.index);
    }

    /**
     * @return the current pipeline (0-9)
     */
    public double getPipeline() {
        return pipeline.getDouble(0);
    }

    /**
     * Sets the pipeline.
     *
     * @param pipeline (0-9)
     */
    public void setPipeline(int pipeline) {
        this.pipeline.setNumber(pipeline);
    }

    /**
     * Takes a snapshot (To test the vision pipelines on stored snapshots).
     */
    public void takeSnapshot() {
        snapshot.setNumber(1);
    }

    private Pose3d getRobotPoseFromJsonDump() {
        final double[] robotPoseArray = getJsonOutput().Results.botpose_wpiblue;
        if (robotPoseArray.length != 6)
            return null;

        return robotPoseArrayToPose2d(robotPoseArray);
    }

    private Pose3d robotPoseArrayToPose2d(double[] robotPoseArray) {
        final Translation3d robotTranslation = new Translation3d(
                robotPoseArray[0],
                robotPoseArray[1],
                robotPoseArray[2]
        );
        final Rotation3d robotRotation = new Rotation3d(
                robotPoseArray[3],
                robotPoseArray[4],
                robotPoseArray[5]
        );

        return new Pose3d(robotTranslation, robotRotation);
    }

    private LimelightJsonOutput getJsonOutput() {
        final String jsonString = json.getString("");
        return JsonHandler.parseJsonStringToObject(
                jsonString,
                LimelightJsonOutput.class
        );
    }

    public enum LedMode {
        USE_LED_MODE(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3);

        public final int index;

        LedMode(int index) {
            this.index = index;
        }

        /**
         * Returns what LedMode has the given index.
         *
         * @param index the index of the LedMode
         * @return the LedMode with the given index. (If there is no LedMode with that index, returns null)
         */
        public static LedMode getLedModeFromValue(int index) {
            return values()[index];
        }
    }

    private static class LimelightJsonOutput {
        private Results Results;

        private static class Results {
            private double[] Classifier;
            private double[] Detector;
            private Fiducial[] Fiducial;
            private double[] retro;
            private double[] botpose, botpose_wpiblue, botpose_wpired;
            private double pID, tl, ts, v;

            private Fiducial getFiducialFromId(int id) {
                for (Fiducial fiducial : Fiducial) {
                    if (fiducial.fID == id)
                        return fiducial;
                }
                return null;
            }

            private static class Fiducial {
                private int fID;
                private String fam;
                private double[] pts;
                private double[] skew;
                private double[] t6c_ts, t6r_fs, t6r_ts, t6t_cs, t6t_rs;
                private double ta, tx, txp, ty, typ;
            }
        }
    }
}

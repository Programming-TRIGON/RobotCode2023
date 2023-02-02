package frc.trigon.robot.posesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.utilities.JsonHandler;

@SuppressWarnings("unused")
public class Limelight implements PoseSource{
    private final String hostname;
    private final NetworkTableEntry tv, tx, ty, ta, botPose, json, ledMode, driverCam, pipeline, snapshot;
    private double lastTimestamp = 0;

    /**
     * Constructs a new Limelight.
     *
     * @param hostname the name of the Limelight
     */
    public Limelight(String hostname) {
        this.hostname = hostname;
        final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(hostname);

        tv = networkTable.getEntry("tv");
        tx = networkTable.getEntry("tx");
        ty = networkTable.getEntry("ty");
        ta = networkTable.getEntry("ta");
        botPose = networkTable.getEntry("botpose_wpiblue");
        json = networkTable.getEntry("json");
        ledMode = networkTable.getEntry("ledMode");
        driverCam = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");
        snapshot = networkTable.getEntry("snapshot");
    }

    @Override
    public boolean canUpdate() {
        return hasNewResult() && getRobotPose() != null;
    }

    @Override
    public boolean hasResults() {
        return tv.getDouble(0) == 1;
    }

    @Override
    public Pose2d getRobotPose() {
        final double[]
                defaultDoubleArray = {1.0},
                robotPoseArray = botPose.getDoubleArray(defaultDoubleArray);

        if (robotPoseArray.length != 6) return null;

        return robotPoseArrayToPose2d(robotPoseArray);
    }

    @Override
    public double getTimestampSeconds() {
        final String jsonString = json.getString("");
        final LimelightJsonOutput limelightJsonOutput = JsonHandler.parseJsonStringToObject(
                json.getString(""),
                LimelightJsonOutput.class
        );


        return limelightJsonOutput.Results.ts;
    }

    @Override
    public double getLastTimestamp() {
        return lastTimestamp;
    }

    @Override
    public void setLastTimestamp(double timestamp) {
        lastTimestamp = timestamp;
    }

    @Override
    public String getName() {
        return hostname;
    }

    /**
     * @return the vertical offset from the crosshair to the target (-20.5 degrees to 20.5 degrees)
     */
    public double getTy() {
        return ty.getDouble(0);
    }

    /**
     * @return the horizontal offset from the crosshair to the target (-27 degrees to 27 degrees)
     */
    public double getTx() {
        return tx.getDouble(0);
    }

    /**
     * @return target's area (0% of image to 100% of image)
     */
    public double getTa() {
        return ta.getDouble(0);
    }

    /**
     * @return true if the driver cam is used, false if the vision cam is used
     */
    public boolean isDriverCam() {
        return driverCam.getDouble(0) == 1;
    }

    /**
     * Sets the driver camera mode.
     *
     * @param useDriverCam true for driver camera, false for vision processing
     */
    public void setDriverCam(boolean useDriverCam) {
        driverCam.setNumber(useDriverCam ? 1 : 0);
    }

    /**
     * @return the current LedMode
     */
    public LedMode getLedMode() {
        return LedMode.getLedModeFromValue(ledMode.getDouble(0));
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

    private Pose2d robotPoseArrayToPose2d(double[] robotPoseArray) {
        final Translation2d robotTranslation = new Translation2d(
                robotPoseArray[0],
                robotPoseArray[1]
        );

        final Rotation2d robotRotation = new Rotation3d(
                robotPoseArray[3],
                robotPoseArray[4],
                robotPoseArray[5]
        ).toRotation2d();

        return new Pose2d(robotTranslation, robotRotation);
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
         * Returns what LedMode has the given value.
         *
         * @param value the value of the LedMode
         * @return the LedMode with the given value. (If there is no LedMode with that value, returns null)
         */
        public static LedMode getLedModeFromValue(double value) {
            for(LedMode currentMode : values()) {
                if(currentMode.index == value) {
                    return currentMode;
                }
            }
            return null;
        }
    }
}

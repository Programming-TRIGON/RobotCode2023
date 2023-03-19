package frc.trigon.robot.components;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ReflectionLimelight implements Sendable {
    private final NetworkTableEntry tv, tx, ty, ts, ta, ledMode, driverCam, pipeline, snapshot;

    /**
     * Constructs a new Limelight.
     *
     * @param hostname the name of the Limelight
     */
    public ReflectionLimelight(String hostname) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(hostname);

        tv = networkTable.getEntry("tv");
        tx = networkTable.getEntry("tx");
        ty = networkTable.getEntry("ty");
        ts = networkTable.getEntry("ts");
        ta = networkTable.getEntry("ta");
        ledMode = networkTable.getEntry("ledMode");
        driverCam = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");
        snapshot = networkTable.getEntry("snapshot");
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
     * @return target's skew (-90 degrees to 0 degrees)
     */
    public double getTs() {
        return ts.getDouble(0);
    }

    /**
     * @return target's area (0% of image to 100% of image)
     */
    public double getTa() {
        return ta.getDouble(0);
    }

    /**
     * @return true if the limelight has any valid targets, false otherwise
     */
    public boolean hasTarget() {
        return tv.getDouble(0) == 1;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("tx", this::getTx, null);
        builder.addDoubleProperty("ty", this::getTy, null);
        builder.addDoubleProperty("ts", this::getTs, null);
        builder.addBooleanProperty("tv", this::hasTarget, null);
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

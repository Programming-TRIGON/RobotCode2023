package frc.trigon.robot.components.cameras;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ReflectionLimelight {
    private final LoggedDashboardNumber tv, tx, ty, ts, ta, ledMode, driverCam, pipeline, snapshot;

    /**
     * Constructs a new Limelight.
     *
     * @param hostname the name of the Limelight
     */
    public ReflectionLimelight(String hostname) {
        tv = new LoggedDashboardNumber(hostname + "/tv");
        tx = new LoggedDashboardNumber(hostname + "/tx");
        ty = new LoggedDashboardNumber(hostname + "/ty");
        ts = new LoggedDashboardNumber(hostname + "/ts");
        ta = new LoggedDashboardNumber(hostname + "/ta");
        ledMode = new LoggedDashboardNumber(hostname + "/ledMode");
        driverCam = new LoggedDashboardNumber(hostname + "/camMode");
        pipeline = new LoggedDashboardNumber(hostname + "/pipeline");
        snapshot = new LoggedDashboardNumber(hostname + "/snapshot");
    }

    /**
     * @return the vertical offset from the crosshair to the target (-20.5 degrees to 20.5 degrees)
     */
    public double getTy() {
        return ty.get();
    }

    /**
     * @return the horizontal offset from the crosshair to the target (-27 degrees to 27 degrees)
     */
    public double getTx() {
        return tx.get();
    }

    /**
     * @return target's skew (-90 degrees to 0 degrees)
     */
    public double getTs() {
        return ts.get();
    }

    /**
     * @return target's area (0% of image to 100% of image)
     */
    public double getTa() {
        return ta.get();
    }

    /**
     * @return true if the limelight has any valid targets, false otherwise
     */
    public boolean hasTarget() {
        return tv.get() == 1;
    }

    /**
     * @return true if the driver cam is used, false if the vision cam is used
     */
    public boolean isDriverCam() {
        return driverCam.get() == 1;
    }

    /**
     * Sets the driver camera mode.
     *
     * @param useDriverCam true for driver camera, false for vision processing
     */
    public void setDriverCam(boolean useDriverCam) {
        driverCam.set(useDriverCam ? 1 : 0);
    }

    /**
     * @return the current LedMode
     */
    public LedMode getLedMode() {
        return LedMode.getLedModeFromValue(ledMode.get());
    }

    /**
     * Sets the led mode.
     *
     * @param mode the wanted LedMode
     */
    public void setLedMode(LedMode mode) {
        ledMode.set(mode.index);
    }

    /**
     * @return the current pipeline (0-9)
     */
    public double getPipeline() {
        return pipeline.get();
    }

    /**
     * Sets the pipeline.
     *
     * @param pipeline (0-9)
     */
    public void setPipeline(int pipeline) {
        this.pipeline.set(pipeline);
    }

    /**
     * Takes a snapshot (To test the vision pipelines on stored snapshots).
     */
    public void takeSnapshot() {
        snapshot.set(1);
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
            for (LedMode currentMode : values()) {
                if (currentMode.index == value) {
                    return currentMode;
                }
            }
            return null;
        }
    }
}

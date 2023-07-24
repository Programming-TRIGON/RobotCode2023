package frc.trigon.robot.robotposesources;

import frc.trigon.robot.components.cameras.FiducialLimelight;

public class AprilTagLimelight extends RobotPoseSourceIO {
    private final FiducialLimelight fiducialLimelight;

    protected AprilTagLimelight(String hostname) {
        fiducialLimelight = new FiducialLimelight(hostname);
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = fiducialLimelight.hasResults();
        inputs.cameraPose = RobotPoseSource.pose3dToDoubleArray(fiducialLimelight.getRobotPoseFromJsonDump());
        inputs.lastResultTimestamp = fiducialLimelight.getLastResultTimestamp();
    }
}

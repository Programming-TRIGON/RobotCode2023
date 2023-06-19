package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.components.FiducialLimelight;

public class AprilTagLimelight extends RobotPoseSource {
    private final FiducialLimelight fiducialLimelight;

    public AprilTagLimelight(String hostname) {
        super(new Transform3d());

        fiducialLimelight = new FiducialLimelight(hostname);
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        if (inputs.name.equals(""))
            inputs.name = fiducialLimelight.getName();

        inputs.hasResult = fiducialLimelight.hasResults();
        inputs.cameraPose = fiducialLimelight.getRobotPoseFromJsonDump();
        inputs.lastResultTimestamp = fiducialLimelight.getLastResultTimestamp();
    }
}

package frc.trigon.robot.robotposesources;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class RobotPoseSource extends SubsystemBase {
    protected final String name;
    private final RobotPoseSourceInputsAutoLogged robotPoseSourceInputs = new RobotPoseSourceInputsAutoLogged();
    private final Transform3d cameraToRobotCenter;
    private RobotPoseSourceIO robotPoseSourceIO;
    private double lastUpdatedTimestamp;
    private Pose2d lastRobotPose = new Pose2d();

    public RobotPoseSource(PoseSourceConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d cameraToRobotCenter) {
        this.cameraToRobotCenter = cameraToRobotCenter;
        this.name = name;
        robotPoseSourceIO = generateIO(robotPoseSourceType);
    }

    protected RobotPoseSource(RobotPoseSourceIO robotPoseSourceIO, String name, Transform3d cameraToRobotCenter) {
        this.cameraToRobotCenter = cameraToRobotCenter;
        this.name = name;
        this.robotPoseSourceIO = robotPoseSourceIO;
    }

    public static double[] pose3dToDoubleArray(Pose3d pose) {
        if (pose == null)
            return null;

        return new double[]{
                pose.getTranslation().getX(),
                pose.getTranslation().getY(),
                pose.getTranslation().getZ(),
                pose.getRotation().getX(),
                pose.getRotation().getY(),
                pose.getRotation().getZ()
        };
    }

    @Override
    public void periodic() {
        robotPoseSourceIO.updateInputs(robotPoseSourceInputs);
        Logger.getInstance().processInputs(name, robotPoseSourceInputs);
    }

    /**
     * @return whether the pose source has a result and that the last updated timestamp is not the current one
     */
    public boolean hasNewResult() {
        return isNewTimestamp() && robotPoseSourceInputs.hasResult;
    }

    /**
     * @return the robot's estimated pose
     */
    public Pose2d getRobotPose() {
        final Pose3d cameraPose = doubleArrayToPose3d(robotPoseSourceInputs.cameraPose);
        if (cameraPose == null)
            return lastRobotPose;

        lastRobotPose = cameraPose.transformBy(cameraToRobotCenter).toPose2d();
        return lastRobotPose;
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    /**
     * @return the last result's timestamp
     */
    public double getLastResultTimestamp() {
        return robotPoseSourceInputs.lastResultTimestamp;
    }

    /**
     * @return the pose source's name
     */
    public String getName() {
        return name;
    }

    protected void setRobotPoseSourceIO(RobotPoseSourceIO robotPoseSourceIO) {
        this.robotPoseSourceIO = robotPoseSourceIO;
    }

    private Pose3d doubleArrayToPose3d(double[] doubleArray) {
        if (doubleArray == null)
            return null;

        return new Pose3d(
                new Translation3d(doubleArray[0], doubleArray[1], doubleArray[2]),
                new Rotation3d(doubleArray[3], doubleArray[4], doubleArray[5])
        );
    }

    private RobotPoseSourceIO generateIO(PoseSourceConstants.RobotPoseSourceType robotPoseSourceType) {
        if (!Robot.IS_REAL)
            return new RobotPoseSourceIO();

        switch (robotPoseSourceType) {
            case LIMELIGHT:
                return new AprilTagLimelight(name);
            case PHOTON_CAMERA:
                return new AprilTagPhotonCamera(name, cameraToRobotCenter);
            default:
                return new RobotPoseSourceIO();
        }
    }
}

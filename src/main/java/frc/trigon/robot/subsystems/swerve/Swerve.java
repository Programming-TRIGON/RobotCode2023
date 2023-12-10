package frc.trigon.robot.subsystems.swerve;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveConstants;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveIO;
import frc.trigon.robot.subsystems.swerve.testingswerve.TestingSwerveConstants;
import frc.trigon.robot.subsystems.swerve.testingswerve.TestingSwerveIO;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveConstants;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveIO;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    private final static Swerve INSTANCE = new Swerve();

    private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveIO swerveIO;
    private final SwerveModuleIO[] modulesIO;
    private final SwerveConstants constants;

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve() {
        swerveIO = generateIO();
        constants = generateConstants();
        modulesIO = getModulesIO();
    }

    @Override
    public void periodic() {
        swerveIO.updateInputs(swerveInputs);
        Logger.getInstance().processInputs("Swerve", swerveInputs);

        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.periodic();

        updateNetworkTables();
    }

    /**
     * @return the swerve constants
     */
    public SwerveConstants getConstants() {
        return constants;
    }

    /**
     * @return the acceleration of the gyro in the z-axis
     */
    public double getGyroZAcceleration() {
        return swerveInputs.accelerationZ;
    }

    /**
     * @return the acceleration of the gyro in the y-axis
     */
    public double getGyroYAcceleration() {
        return swerveInputs.accelerationY;
    }

    /**
     * @return the acceleration of the gyro in the x-axis
     */
    public double getGyroXAcceleration() {
        return swerveInputs.accelerationX;
    }

    /**
     * @return the heading of the robot
     */
    public Rotation2d getHeading() {
        final double heading = swerveInputs.gyroAngleDegrees;
        final double inputtedHeading = MathUtil.inputModulus(heading, -180, 180);

        return Rotation2d.fromDegrees(inputtedHeading);
    }

    /**
     * @return the robot's current velocity
     */
    public ChassisSpeeds getCurrentVelocity() {
        return constants.getKinematics().toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the heading of the robot.
     *
     * @param heading the new heading
     */
    public void setHeading(Rotation2d heading) {
        swerveIO.setHeading(heading);
    }

    /**
     * @return the pitch of the swerve, in degrees
     */
    public double getPitch() {
        return swerveInputs.gyroPitchDegrees;
    }

    /**
     * @return the swerve's module's positions
     */
    public SwerveModulePosition[] getModulePositions() {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getCurrentPosition();

        return swerveModulePositions;
    }

    /**
     * Locks the swerve, so it'll be hard to move it.
     */
    void lockSwerve() {
        setBrake(true);
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        modulesIO[0].setTargetState(right);
        modulesIO[1].setTargetState(left);
        modulesIO[2].setTargetState(left);
        modulesIO[3].setTargetState(right);
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    void selfRelativeDrive(Translation2d translation, Rotation2d rotation) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Drives the swerve with the given velocities, relative to the field's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    void fieldRelativeDrive(Translation2d translation, Rotation2d rotation) {
        final Rotation2d heading = AllianceUtilities.isBlueAlliance() ? getHeading() : getHeading().plus(Rotation2d.fromRotations(0.5));

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians(),
                heading
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Sets whether the swerve drive should be in closed loop control, or in open loop control.
     *
     * @param closedLoop true if the drive motor should be in closed loop control, false if it should be in open loop control
     */
    void setClosedLoop(boolean closedLoop) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setDriveMotorClosedLoop(closedLoop);
    }

    /**
     * Stops the swerve's motors.
     */
    void stop() {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.stop();
    }

    /**
     * Sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    void setBrake(boolean brake) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setBrake(brake);
    }

    /**
     * Sets the swerve's target module states.
     *
     * @param swerveModuleStates the target module states
     */
    void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i].setTargetState(swerveModuleStates[i]);
    }

    private void updateNetworkTables() {
        Logger.getInstance().recordOutput("Swerve/Velocity/rot", getCurrentVelocity().omegaRadiansPerSecond);
        Logger.getInstance().recordOutput("Swerve/Velocity/x", getCurrentVelocity().vxMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Velocity/y", getCurrentVelocity().vyMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/moduleStates", getModuleStates());
        Logger.getInstance().recordOutput("Swerve/targetStates", getTargetModuleStates());
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = constants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getCurrentState();

        return states;
    }

    private SwerveModuleState[] getTargetModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getTargetState();

        return states;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return
                Math.abs(chassisSpeeds.vxMetersPerSecond) <= constants.getDriveNeutralDeadband() &&
                        Math.abs(chassisSpeeds.vyMetersPerSecond) <= constants.getDriveNeutralDeadband() &&
                        Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= constants.getRotationNeutralDeadband();
    }

    private SwerveConstants generateConstants() {
        switch (ConfigurationConstants.ROBOT_TYPE) {
            case TRIHARD:
                return new TrihardSwerveConstants();
            case TESTING:
                return new TestingSwerveConstants();
            default:
                return new SimulationSwerveConstants();
        }
    }

    private SwerveModuleIO[] getModulesIO() {
        if (ConfigurationConstants.IS_REPLAY) {
            final SwerveModuleIO[] modulesIO = new SwerveModuleIO[4];
            modulesIO[0] = new SwerveModuleIO("FRONT_LEFT");
            modulesIO[1] = new SwerveModuleIO("FRONT_RIGHT");
            modulesIO[2] = new SwerveModuleIO("REAR_LEFT");
            modulesIO[3] = new SwerveModuleIO("REAR_RIGHT");
            return modulesIO;
        }

        return constants.getModulesIO();
    }

    private SwerveIO generateIO() {
        if (ConfigurationConstants.IS_REPLAY)
            return new SwerveIO();

        switch (ConfigurationConstants.ROBOT_TYPE) {
            case TRIHARD:
                return new TrihardSwerveIO();
            case TESTING:
                return new TestingSwerveIO();
            default:
                return new SimulationSwerveIO();
        }
    }
}


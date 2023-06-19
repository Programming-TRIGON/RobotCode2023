package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class SwerveIO {
    @AutoLog
    protected static class SwerveInputs {
        public double gyroAngleDegrees = 0;
        public double gyroPitchDegrees = 0;
        public double accelerationX = 0;
        public double accelerationY = 0;
        public double accelerationZ = 0;
    }

    /**
     * Updates the swerve's inputs.
     *
     * @param inputs the inputs to update
     */
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
    }

    /**
     * Sets the heading of the robot.
     *
     * @param heading the new heading
     */
    protected void setHeading(Rotation2d heading) {
    }
}

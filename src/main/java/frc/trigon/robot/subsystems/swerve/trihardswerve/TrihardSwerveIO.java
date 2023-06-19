package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class TrihardSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = TrihardSwerveConstants.GYRO;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        inputs.gyroAngleDegrees = gyro.getRotation2d().getDegrees();
        inputs.gyroPitchDegrees = gyro.getPitch().getValue();
        inputs.accelerationX = gyro.getAccelerationX().getValue();
        inputs.accelerationY = gyro.getAccelerationY().getValue();
        inputs.accelerationZ = gyro.getAccelerationZ().getValue();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        // TODO: check if this is degrees or something else
        gyro.setYaw(heading.getDegrees());
    }
}

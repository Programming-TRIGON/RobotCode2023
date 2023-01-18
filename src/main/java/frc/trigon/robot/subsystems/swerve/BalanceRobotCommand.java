package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;


public class BalanceRobotCommand extends CommandBase {

    private Swerve swerve = Swerve.getInstance();
    private Pigeon2 gyro = new Pigeon2(9);

    private double Kp = 0,
            Ki = 0,
            Kd = 0;
    PIDController balancePidController = new PIDController(Kp, Ki, Kd);

    public BalanceRobotCommand() {
        addRequirements(swerve);
        balancePidController.reset();
        balancePidController.setSetpoint(0);
    }

    /***
     * moves the robot back and forth to center the robot
     */
    private void autoBalance() {
        if (gyro.getPitch() > 1 || gyro.getPitch() <= -1) {
            FieldRelativeX(() -> calculateOutput());
        }
    }

    /***
     * @param x moves forward or backwards in meters according to the field
     */
    private void FieldRelativeX(DoubleSupplier x) {
        swerve.selfRelativeDrive(
                new Translation2d(x.getAsDouble(), 0),
                new Rotation2d(0));
    }

    /**
     * @return the output in meter(used for centring)
     */
    private double calculateOutput() {
        return cmToMeter(100 * balancePidController.
                calculate(-gyro.getPitch() / 100));
    }

    private double cmToMeter(double cm) {
        return cm / 100;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        autoBalance();
    }
}


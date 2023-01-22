package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;


public class BalanceRobotCommand extends CommandBase {

    private final Swerve swerve = Swerve.getInstance();
    private final Pigeon2 gyro = new Pigeon2(9);

    private final double P = 0;
    private final double I = 0;
    private final double D = 0;

    PIDController balancePidController = new PIDController(P, I, D);

    public BalanceRobotCommand() {
        addRequirements(swerve);
        balancePidController.reset();
        balancePidController.setSetpoint(0);
    }

    private void autoBalance() {
        if (gyro.getPitch() > 1 || gyro.getPitch() <= -1) {
            selfBalanceDrive(() -> calculateOutput());
        }
    }

    /**
     * @param x moves forward or backwards in meters according to the field
     */
    private void selfBalanceDrive(DoubleSupplier x) {
        swerve.selfRelativeDrive(
                new Translation2d(x.getAsDouble(), 0), new Rotation2d(0));
    }

    /**
     * @return the output in meter(used for centering)
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
        addRequirements(swerve);
        balancePidController.reset();
        balancePidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        autoBalance();
    }
}


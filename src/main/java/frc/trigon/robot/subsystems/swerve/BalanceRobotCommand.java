package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class BalanceRobotCommand extends CommandBase {

    private final Swerve swerve = Swerve.getInstance();

    private final double P = 0.1;
    private final double I = 0.1;
    private final double D = 0.1;

    PIDController balancePidController = new PIDController(P, I, D);

    public BalanceRobotCommand() {
        addRequirements(swerve);
        balancePidController.reset();
        balancePidController.setSetpoint(0);
        balancePidController.setTolerance(0.5, 0.5);
    }

    /**
     * @param x moves forward or backwards in meters according to the field
     */
    private void selfBalanceDrive(double x) {
        swerve.selfRelativeDrive(
                new Translation2d(x, 0), new Rotation2d(0));
    }


    @Override
    public void initialize() {
        addRequirements(swerve);
        balancePidController.reset();
        balancePidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        selfBalanceDrive(balancePidController.calculate(swerve.getPitch()));
    }

    @Override
    public boolean isFinished() {
        return balancePidController.atSetpoint();
    }
}


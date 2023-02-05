package frc.trigon.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        Logger.configureLoggingAndConfig(robotContainer, false);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Logger.updateEntries();
    }

    @Override
    public void disabledInit() {
        SwerveCommands.getSetSwerveBrakeCommand(true)
                .andThen(new WaitCommand(1.5))
                .andThen(SwerveCommands.getSetSwerveBrakeCommand(false)).schedule();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}

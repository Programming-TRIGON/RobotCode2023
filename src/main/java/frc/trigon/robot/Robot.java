package frc.trigon.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.FilesHandler;
import io.github.oblarg.oblog.Logger;

import java.io.IOException;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private CommandBase autonomousCommand;
    public static final boolean IS_REAL = Robot.isReal();


    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        PathPlannerServer.startServer(5811);
        Logger.configureLoggingAndConfig(robotContainer, false);
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Logger.updateEntries();
    }

    @Override
    public void disabledInit() {
        SwerveCommands.getBrakeAndCoastCommand().schedule();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();//.withTimeout(15).andThen(Commands.fakeStaticColor(Color.kDarkGoldenrod));
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.teleopInit();
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

    private void setDeployFolderToMaxPermissions() {
        try {
            FilesHandler.setDeployFolderPermissions(true, true, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

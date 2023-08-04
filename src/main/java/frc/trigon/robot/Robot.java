package frc.trigon.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.constants.BuildConstants;
import frc.trigon.robot.constants.ConfigurationConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.FilesHandler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.IOException;

public class Robot extends LoggedRobot {
    public static final boolean IS_REAL = Robot.isReal();
    private final Logger logger = Logger.getInstance();
    private RobotContainer robotContainer;
    private CommandBase autonomousCommand;

    @Override
    public void robotInit() {
//        if (IS_REAL)
//            setDeployFolderToMaxPermissions();
        robotContainer = new RobotContainer();
        configLogger();
        recordBuild();

        PathPlannerServer.startServer(5811);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
        autonomousCommand.cancel();
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
            FilesHandler.setPermissions(Filesystem.getDeployDirectory(), true, true, true);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void configLogger() {
        if (ConfigurationConstants.IS_REPLAY) {
            setUseTiming(false);
            final String logPath = LogFileUtil.findReplayLog();
            logger.setReplaySource(new WPILOGReader(logPath));
            logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));

            logger.start();
            return;
        }

        switch (ConfigurationConstants.ROBOT_TYPE) {
            case TRIHARD:
            case TESTING:
                logger.addDataReceiver(new WPILOGWriter(OperatorConstants.REAL_MODE_LOG_DIRECTORY));
                logger.addDataReceiver(new NT4Publisher());
                break;
            case SIMULATION:
                logger.addDataReceiver(new WPILOGWriter(OperatorConstants.SIMULATION_MODE_LOG_DIRECTORY));
                logger.addDataReceiver(new NT4Publisher());
        }

        logger.start();
    }

    private void recordBuild() {
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
    }
}

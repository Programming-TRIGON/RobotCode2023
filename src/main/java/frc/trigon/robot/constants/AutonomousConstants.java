package frc.trigon.robot.constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.FilesHandler;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AutonomousConstants {
    public static final HashMap<String, Command> EVENT_MAP = new HashMap<>();
    public static final PathConstraints AUTONOMOUS_PATH_CONSTRAINS = new PathConstraints(2, 1.5);
    public static final List<String> AUTONOMOUS_PATHS_NAMES = new ArrayList<>();
    public static final HashMap<String, List<PathPlannerTrajectory>> PRELOADED_PATHS = new HashMap<>();
    private static final File PATH_PLANNER_DIRECTORY = new File(FilesHandler.DEPLOY_PATH + "pathplanner");

    static {
        configureEventMap();
        configureAutonomousPathsNames();
    }

    private static void configureEventMap() {
        EVENT_MAP.put(
                "place-cone-2", Commands.getPlaceConeAtHybridCommand()
        );
        EVENT_MAP.put(
                "place-cone-3", Commands.getPlaceConeAtHighForAutoCommand()
        );
        EVENT_MAP.put(
                "collect", RobotContainer.GRIPPER.getCollectCommand().alongWith(RobotContainer.ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED_COLLECTING))
        );
        EVENT_MAP.put(
                "blind-collect", RobotContainer.GRIPPER.getCollectCommand().alongWith(RobotContainer.ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED_COLLECTING, false, 1, 1))
        );
        EVENT_MAP.put(
                "close-collect", RobotContainer.GRIPPER.getHoldCommand().alongWith(RobotContainer.ARM.getGoToStateCommand(ArmConstants.ArmStates.CLOSED, true, 1.5, 1.5))
        );
        EVENT_MAP.put(
                "prepare-collect", RobotContainer.ARM.getGoToPositionCommand(-35, ArmConstants.ArmStates.CLOSED_COLLECTING.secondMotorPosition+20, false, 0.7, 0.7)
        );
        EVENT_MAP.put(
                "place-cube-2", Commands.getPlaceCubeAtMidForAutoCommand()
        );
        EVENT_MAP.put(
                "place-cube-3", Commands.getPlaceCubeAtHighCommandForAuto()
        );
        EVENT_MAP.put(
                "balance", SwerveCommands.getBalanceCommand()
        );
        EVENT_MAP.put(
                "prepare-cube-3", Arm.getInstance().getGoToStateCommand(ArmConstants.ArmStates.CUBE_HIGH)
        );
        EVENT_MAP.put(
                "eject", Gripper.getInstance().getEjectCommand()
        );
        EVENT_MAP.put(
                "quick-eject", Gripper.getInstance().getFullEjectCommand().withTimeout(0.5)
        );
    }

    private static void configureAutonomousPathsNames() {
        if (!PATH_PLANNER_DIRECTORY.exists())
            return;

        final File[] files = PATH_PLANNER_DIRECTORY.listFiles();
        if (files == null)
            return;

        for (File file : files) {
            if (!file.isFile() || !file.getName().endsWith(".path"))
                continue;

            AUTONOMOUS_PATHS_NAMES.add(file.getName().substring(0, file.getName().length() - 5));
        }
    }
}

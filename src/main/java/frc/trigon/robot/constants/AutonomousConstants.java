package frc.trigon.robot.constants;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.utilities.FilesHandler;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AutonomousConstants {
    public static final HashMap<String, Command> EVENT_MAP = new HashMap<>();
    public static final PathConstraints AUTONOMOUS_PATH_CONSTRAINS = new PathConstraints(3, 2);
    public static final List<String> AUTONOMOUS_PATHS_NAMES = new ArrayList<>();
    private static final File PATH_PLANNER_DIRECTORY = new File(FilesHandler.DEPLOY_PATH + "pathplanner");

    static {
        configureEventMap();
        configureAutonomousPathsNames();
    }

    private static void configureEventMap() {
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

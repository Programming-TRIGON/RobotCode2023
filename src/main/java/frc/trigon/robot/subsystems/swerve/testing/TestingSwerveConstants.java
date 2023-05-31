package frc.trigon.robot.subsystems.swerve.testing;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.utilities.FilesHandler;

import java.io.File;

public class TestingSwerveConstants {
    static final double BRAKE_TIME_SECONDS = 0.3;
    static final File SWERVE_PATH = new File(FilesHandler.DEPLOY_PATH + "testingswerve/");
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.21564, 2.7054, 0.38437);
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0,
            ROTATION_NEUTRAL_DEADBAND = 0;
    static final boolean
            USE_HEADING_CORRECTION_FOR_NORMAL_DRIVE = true,
            USE_HEADING_CORRECTION_FOR_AUTO = true;
    static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(12, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(15, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(15, 0, 0);
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            720,
            1200
    );
    static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            ROTATION_PID_CONSTANTS.kP,
            ROTATION_PID_CONSTANTS.kI,
            ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    static final double STOPPING_ACCELERATION = 4500;
    static final double
            TRANSLATION_TOLERANCE = 0.2,
            ROTATION_TOLERANCE = 1,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;

    static {
        ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
    }
}

package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final double
            FIELD_LENGTH_METERS = 16.54175,
            FIELD_WIDTH_METERS = 8.02;
    public static final Rotation2d FIELD_MIRRORING_LINE = Rotation2d.fromDegrees(90);
    private static final double IN_FRONT_OF_GRID_X = 1.83;
    private static final Rotation2d IN_FRONT_OF_GRID_ROTATION = Rotation2d.fromRotations(180);

    public enum GridAlignment {
        LEFT_GRID_LEFT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, 5.05, IN_FRONT_OF_GRID_ROTATION)),
        LEFT_GRID_SHELF(new Pose2d(IN_FRONT_OF_GRID_X, 4.42, IN_FRONT_OF_GRID_ROTATION)),
        LEFT_GRID_RIGHT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, 3.86, IN_FRONT_OF_GRID_ROTATION)),
        MIDDLE_GRID_LEFT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, 3.30, IN_FRONT_OF_GRID_ROTATION)),
        MIDDLE_GRID_SHELF(new Pose2d(IN_FRONT_OF_GRID_X, 2.74, IN_FRONT_OF_GRID_ROTATION)),
        MIDDLE_GRID_RIGHT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, 2.18, IN_FRONT_OF_GRID_ROTATION)),
        RIGHT_GRID_LEFT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, 1.62, IN_FRONT_OF_GRID_ROTATION)),
        RIGHT_GRID_SHELF(new Pose2d(IN_FRONT_OF_GRID_X, 1.06, IN_FRONT_OF_GRID_ROTATION)),
        RIGHT_GRID_RIGHT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, 0.5, IN_FRONT_OF_GRID_ROTATION));

        public final Pose2d inFrontOfGridPose;

        GridAlignment(Pose2d inFrontOfGridPose) {
            this.inFrontOfGridPose = inFrontOfGridPose;
        }
    }
}

package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.robotposesources.PoseSourceConstants;

public class FieldConstants {
    public static final double
            FIELD_LENGTH_METERS = 16.54175,
            FIELD_WIDTH_METERS = 8.02;
    private static final double BUMPERS_LENGTH_METERS = 0.86;
    public static final Pose2d IN_FRONT_OF_FEEDER_POSE = new Pose2d(14.2256, 8.01668 - (BUMPERS_LENGTH_METERS / 2), Rotation2d.fromDegrees(90));
    private static final double IN_FRONT_OF_GRID_X = 1.834;
    private static final double RAMP_DISTANCE_FROM_SHELF = 2.748-2.202;
    private static final double
            LEFT_GRID_SHELF_Y = PoseSourceConstants.TAGS_ID_TO_POSE.get(6).getY(),
            MIDDLE_GRID_SHELF_Y = PoseSourceConstants.TAGS_ID_TO_POSE.get(7).getY(),
            RIGHT_GRID_SHELF_Y = PoseSourceConstants.TAGS_ID_TO_POSE.get(8).getY();
    private static final Rotation2d IN_FRONT_OF_GRID_ROTATION = Rotation2d.fromRotations(0.5);

    public enum GridAlignment {
        LEFT_GRID_LEFT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, LEFT_GRID_SHELF_Y + RAMP_DISTANCE_FROM_SHELF, IN_FRONT_OF_GRID_ROTATION)),
        LEFT_GRID_SHELF(new Pose2d(IN_FRONT_OF_GRID_X, LEFT_GRID_SHELF_Y, IN_FRONT_OF_GRID_ROTATION)),
        LEFT_GRID_RIGHT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, LEFT_GRID_SHELF_Y - RAMP_DISTANCE_FROM_SHELF, IN_FRONT_OF_GRID_ROTATION)),
        MIDDLE_GRID_LEFT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, MIDDLE_GRID_SHELF_Y + RAMP_DISTANCE_FROM_SHELF, IN_FRONT_OF_GRID_ROTATION)),
        MIDDLE_GRID_SHELF(new Pose2d(IN_FRONT_OF_GRID_X, MIDDLE_GRID_SHELF_Y, IN_FRONT_OF_GRID_ROTATION)),
        MIDDLE_GRID_RIGHT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, MIDDLE_GRID_SHELF_Y - RAMP_DISTANCE_FROM_SHELF, IN_FRONT_OF_GRID_ROTATION)),
        RIGHT_GRID_LEFT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, RIGHT_GRID_SHELF_Y + RAMP_DISTANCE_FROM_SHELF, IN_FRONT_OF_GRID_ROTATION)),
        RIGHT_GRID_SHELF(new Pose2d(IN_FRONT_OF_GRID_X, RIGHT_GRID_SHELF_Y, IN_FRONT_OF_GRID_ROTATION)),
        RIGHT_GRID_RIGHT_RAMP(new Pose2d(IN_FRONT_OF_GRID_X, RIGHT_GRID_SHELF_Y - RAMP_DISTANCE_FROM_SHELF, IN_FRONT_OF_GRID_ROTATION));

        public final Pose2d inFrontOfGridPose;
        public final int gridNumber;
        public final int columnNumber;

        GridAlignment(Pose2d inFrontOfGridPose) {
            this.inFrontOfGridPose = inFrontOfGridPose;
            final int index = ordinal();

            gridNumber = index / 3 + 1;
            columnNumber = index % 3 + 1;
        }

        public static GridAlignment getGridAlignment(int gridNumber, int columnNumber) {
            return values()[(gridNumber - 1) * 3 + columnNumber - 1];
        }
    }
}

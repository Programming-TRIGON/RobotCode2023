package frc.trigon.robot.subsystems.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class PoseLimiter {
    private final double robotLength;
    private final StraightLine2d[] limitLines;

    public PoseLimiter(double robotLength, StraightLine2d... limitLines) {
        this.limitLines = limitLines;
        this.robotLength = robotLength;
    }

    public Pose2d limitPose(Pose2d pose2d) {
        final List<StraightLine2d> touchedLimitLines = getTouchedLimitLines(pose2d);

        if (touchedLimitLines.isEmpty())
            return pose2d;

        double newX = Double.NaN, newY = Double.NaN;

        for (StraightLine2d touchedLimitLine : touchedLimitLines) {
            if (touchedLimitLine.isVertical)
                newX = getClosestNumber(pose2d.getX(), touchedLimitLine.minX + (robotLength / 2), touchedLimitLine.minX - (robotLength / 2));
            else
                newY = getClosestNumber(pose2d.getY(), touchedLimitLine.minY + (robotLength / 2), touchedLimitLine.minY - (robotLength / 2));
        }

        if (Double.isNaN(newX))
            newX = pose2d.getX();

        if (Double.isNaN(newY))
            newY = pose2d.getY();

        return new Pose2d(newX, newY, pose2d.getRotation());
    }

    private double getClosestNumber(double number, double... numbers) {
        double closestNumber = numbers[0];
        for (int i = 1; i < numbers.length; i++) {
            if (Math.abs(numbers[i] - number) < Math.abs(closestNumber - number))
                closestNumber = numbers[i];
        }

        return closestNumber;
    }

    private List<StraightLine2d> getTouchedLimitLines(Pose2d pose2d) {
        final List<StraightLine2d> touchedLimitLines = new ArrayList<>();

        for (StraightLine2d limitLine : limitLines) {
            if (limitLine.isTouchingLine(pose2d, robotLength / 2))
                touchedLimitLines.add(limitLine);
        }

        return touchedLimitLines;
    }

    public static class StraightLine2d {
        private final boolean isVertical;
        private final double maxX, minX, maxY, minY;

        public StraightLine2d(Translation2d minTranslation, Translation2d maxTranslation) {
            minX = minTranslation.getX();
            maxX = maxTranslation.getX();
            minY = minTranslation.getY();
            maxY = maxTranslation.getY();

            isVertical = minX == maxX;
        }

        public boolean isTouchingLine(Pose2d pose2d, double tolerance) {
            if (isVertical)
                return Math.abs(pose2d.getX() - minX) <= tolerance && pose2d.getY() >= minY && pose2d.getY() <= maxY;
            else
                return Math.abs(pose2d.getY() - minY) <= tolerance && pose2d.getX() >= minX && pose2d.getX() <= maxX;
        }
    }
}

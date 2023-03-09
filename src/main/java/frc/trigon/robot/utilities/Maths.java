package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Maths {
    /**
     * Calculates the polynomial of the form ax^2 + bx + c.
     *
     * @param a the a in the polynomial
     * @param b the b in the polynomial
     * @param c the c in the polynomial
     * @param x the x in the polynomial
     * @return the result of the polynomial
     */
    public static double calculatePolynomial(double a, double b, double c, double x) {
        return (a * Math.pow(x, 2)) + (b * x) + c;
    }

    public static double calculateSlope(Translation2d firstPoint, Translation2d secondPoint) {
        return (firstPoint.getY() - secondPoint.getY()) / (firstPoint.getX() - secondPoint.getX());
    }

    public static Rotation2d getAngleBetweenTranslations(Translation2d firstPoint, Translation2d secondPoint) {
        final double slope = calculateSlope(firstPoint, secondPoint);
        return new Rotation2d(Math.atan(slope));
    }
}

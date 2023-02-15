package frc.trigon.robot.utilities;

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
}

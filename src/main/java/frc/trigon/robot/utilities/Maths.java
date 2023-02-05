package frc.trigon.robot.utilities;

import java.util.List;

public class Maths {
    public static double average(List<Double> values) {
        return average(values.stream().mapToDouble(Double::doubleValue).toArray());
    }

    public static double average(double... values) {
        final double sum = sum(values);
        return sum / values.length;
    }

    public static double sum(double... values) {
        double sum = 0;

        for (double value : values)
            sum += value;

        return sum;
    }
}

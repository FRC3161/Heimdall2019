package frc.robot;

public class MathUtils {
    private MathUtils() { }

    public static double absClamp(double value, double clamp) {
        if (Math.abs(value) <= clamp) {
            return value;
        }
        if (value > 0) {
            return clamp;
        } else {
            return -clamp;
        }
    }
}
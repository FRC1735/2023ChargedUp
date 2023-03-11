package frc.robot;

public class Utils {
    private static double CLOSE_ENOUGH = 0.01;

    public static boolean isCloseEnough(double value, double target) {
        return value > target - CLOSE_ENOUGH && value < target + CLOSE_ENOUGH;
    }
}

package frc.robot.util.math;

import org.opencv.core.Point;

public class MathFunctions {
    public static double angleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;
    }

    /**
     * Constrains a number between a min and max value. Functionally
     * equivalent to std::clamp.
     * 
     * @param n Number to be clipped
     * @param min Lower bound
     * @param max Upper bound
     * @return Clipped value
     */
    public static double clip(double n, double min, double max) {
        return Math.max(min, Math.min(max, n));
    }

    public static double distance2D(Point p1, Point p2) {
        return Math.hypot(Math.abs(p2.x - p1.x), Math.abs(p2.y - p1.y));
    }

    /**
     * Converts feet to meters
     * 
     * @param feet Measurement in feet
     * @return Measurement in meters
     */
    public static double toMeters(double feet) {
        return feet * 0.3048;
    }

    /**
     * Converts meters to feet
     * 
     * @param meters Measurement in meters
     * @return Measurement in feet
     */
    public static double toFeet(double meters) {
        return meters * 3.28084;
    }
}

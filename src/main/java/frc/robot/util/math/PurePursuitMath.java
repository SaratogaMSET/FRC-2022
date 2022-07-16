package frc.robot.util.math;

import org.opencv.core.Point;

public class PurePursuitMath {
    public static double angleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;

        /* 
        angle = angle % (2 * Math.PI);
        angle = (angle + (2 * Math.PI)) % (2 * Math.PI);
        if (angle >= Math.PI) {
            angle -= (2 * Math.PI);
        }

        return angle;
        */
    }

    public static double clip(double n, double min, double max) {
        return Math.max(min, Math.min(max, n));
    }

    public static double distance2D(Point p1, Point p2) {
        return Math.hypot(Math.abs(p2.x - p1.x), Math.abs(p2.y - p1.y));
    }
}

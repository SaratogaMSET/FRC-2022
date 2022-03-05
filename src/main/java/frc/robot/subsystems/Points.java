package frc.robot.subsystems;
 
import java.util.ArrayList;
import java.util.*;
import java.lang.*;
import java.lang.Comparable;

public class Points { // implements Comparable {
    private double velocity;
    private double distance;
    private double[] point = new double[2];

    public Points(double distance1, double velocity1) {
        point[0] = distance1;
        point[1] = velocity1;
    }

    public double getVelocity() {
        return this.velocity;
    }

    public double getDistance() {
        return this.distance;
    }


    // public int compareTo(Points other) {
    //     int otherDistance = other.getDistance();
    //     return this.getDistance() - otherDistance;
    // }

    public boolean lessThan(Points other) {
        double otherDistance = other.getDistance();
        if(otherDistance < this.distance){
            return true;
        }

        else return false;
    }

    // @Override
    // public int compare(Points other) {
    //     double otherDistance = other.getDistance();
    //     if (otherDistance == this.distance)
    //         return 0;
    //     else if (otherDistance > this.distance)
    //         return 1;
    //     else
    //         return -1;
    // }



}

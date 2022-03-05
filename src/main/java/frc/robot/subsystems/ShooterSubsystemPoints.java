package frc.robot.subsystems;
 
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.util.drivers.LazyTalonFX;



public class ShooterSubsystemPoints extends SubsystemBase{
    public LazyTalonFX shooterMotor;
    private LazyTalonFX shooterMotor2;

    private ArrayList<Points> points = new ArrayList<Points>(100);



    public ShooterSubsystemPoints() {
        
        shooterMotor = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
        shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.LS_MOTOR);
    }
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
    }
    public void run(double d) {
        shooterMotor.set(TalonFXControlMode.PercentOutput,getVelocity(d));
        shooterMotor2.set(TalonFXControlMode.PercentOutput,-getVelocity(d));
    }

    public double getVelocity(double distance) {
        for (int i = 1; i < points.size(); i++) {
            if(points.get(i).getDistance() < distance){
                continue;
            }
            else {
                double[] equation = lineFromPoints(points.get(i-1).getDistance(), points.get(i-1).getVelocity(), points.get(i).getDistance(), points.get(i).getVelocity());
                return equation[0] * distance + equation[1];
            }
        }

        return 0.0;
    }

    public double[] lineFromPoints(double distance1, double velocity1, double distance2, double velocity2) //, Pair P, Pair Q)
    {
        double a = velocity2 - velocity1; // int a = Q.second - P.second;
        double b = distance2 - distance1; // int b = P.first - Q.first;
        double m = (double) a/b;
        double c = (double) velocity1 - m*distance1;

        double[] equationValues = {m, c};
 
        return equationValues;
    }

    public ArrayList<Points> sort(ArrayList<Points> points) 
    {
        // for(int i=0; i< points.size(); i++){
        //     points.get(i);
        // }

        for (int i = 0; i < points.size(); i++) {
 
            // Inner nested loop pointing 1 index ahead
            for (int j = i + 1; j < points.size(); j++) {
 
                // Checking elements
                
                if (points.get(j).lessThan(points.get(i))) {
 
                    // Swapping
                    Points temp = points.get(i);
                    points.set(i, points.get(j));
                    points.set(j,temp);
                }
            }
 
            // Printing sorted array elements
        }

        return points;
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionSystem extends SubsystemBase {

  public static enum VisionState {
    NO_TARGET, TARGET_VISIBLE
  }

  NetworkTable table;
  NetworkTableEntry tx, ty, tv;

  double x, y, v;

  public VisionSystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    table.getEntry("pipeline").setNumber(0);

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightTargets", v);
  }

  public VisionState updateVisionState() {
    if (v == 1) {
      return VisionState.TARGET_VISIBLE;
    }
    return VisionState.NO_TARGET;
  }

  // public String getPath() {
  //   if (v == 0) {
  //     SmartDashboard.putString("Path", "Blue");
  //     return ("Blue");
  //   } else if (x > -3 && x < 3) {
  //     SmartDashboard.putString("Path", "Red");
  //     return ("Red");
  //   } else {
  //     SmartDashboard.putString("Path", "Not Found");
  //     return ("Not Found");
  //   }
  // }

  // public ShooterState getShooterStateFromDistance() {
  //   double distance = getDistance();
  //   if (distance < Constants.Vision.Distance.GREEN) {
  //     return ShooterState.GREEN;
  //   } else if (distance < Constants.Vision.Distance.YELLOW) {
  //     return ShooterState.YELLOW;
  //   } else if (distance < Constants.Vision.Distance.BLUE) {
  //     return ShooterState.BLUE;
  //   } else if (distance < Constants.Vision.Distance.RED) {
  //     return ShooterState.RED;
  //   }
  //   return ShooterState.IDLE;
  // }
  
  // public double getDistance() { // distance
  //   return (Constants.Vision.H2 - Constants.Vision.H1) / Math.tan(Math.toRadians(Constants.Vision.A1 + y));
  // }

  public double getTx() {
    return x;
  }
}

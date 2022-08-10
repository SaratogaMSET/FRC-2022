// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  private NetworkTable table;
  private NetworkTableEntry tx, ty, tv;
  private static double x, y, v;
  private Pose2d m_pose;

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    m_pose = new Pose2d(new Translation2d(), new Rotation2d());
  }

  public static enum VisionState {
    NO_TARGET, TARGET_VISIBLE, TARGET_ACQUIRED
  }
  
  public void refresh() {
    if (table == null) {
      return;
    }

    table.getEntry("pipeline").setNumber(0);

    // ta = table.getEntry("ta"); // area of target visible
    tx = table.getEntry("tx"); // horizontal offset
    ty = table.getEntry("ty"); // vertical offset
    tv = table.getEntry("tv"); // valid target? (0 or 1)

    // a = ta.getDouble(0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);

    // Pose estimation
    double[] camtran = table.getEntry("camtran").getDoubleArray(new double[]{});
    // Translation2d tranToGoal = new Translation2d(camtran[2], camtran[0] * -1); // potential FIXME
    Translation2d tranToGoal = new Translation2d(camtran[0], camtran[1]); // potential FIXME
    Rotation2d rotToGoal = new Rotation2d(camtran[4] * 1); // potential FIXME
    m_pose = new Pose2d(tranToGoal, rotToGoal);
  }

  public VisionState updateVisionState(){
    if(v == 1) {
      return VisionState.TARGET_VISIBLE;
    }
    return VisionState.NO_TARGET;
  }

  public double getDistanceFromTarget() {
    if (table == null) {
      return -1.0;
    }

    return (Constants.Vision.H2 - Constants.Vision.H1) / Math.tan(Math.toRadians(Constants.Vision.A1 + y));
  }

  public double getRawAngle() {
    return -x;
    // return 15;
  }

  public Pose2d getCamPose() {
    return m_pose;
  }

  public double getLatency() {
    return table.getEntry("tl").getDouble(0) + 11;
  }

  // private void updateSmartDashboard() {
  //   double temp_x = Math.abs(x);
  //   if(a >= Constants.Vision.AREA_VISIBLE) {
  //     if (temp_x <= Constants.Vision.Angle.ON_TARGET_X) {
  //       SmartDashboard.putString("angle", "on target");
  //     } else {
  //       SmartDashboard.putString("angle", "visible");
  //     }
  //   } else {
  //     SmartDashboard.putString("angle", "off");
  //   }
  // } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void periodic() {
    refresh();
    updateVisionState();
  }
}
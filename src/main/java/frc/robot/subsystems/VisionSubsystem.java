// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // private final I2C.Port i2cPort = I2C.Port.kOnboard; //Constants.Colorwheel.COLOR_SENSOR; 
  private NetworkTable table;
  private NetworkTableEntry ta, tx, ty, tv;
  private static double a, x, y, v;

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    //SmartDashboard.putString("Test", "Test");
    //SmartDashboard.putString("Test2", "Test2");
  }

  public static enum VisionState {
    NO_TARGET, TARGET_VISIBLE, TARGET_ACQUIRED
  }
  
  public void refresh() {
    if (table == null) {
      return;
    }

    table.getEntry("pipeline").setNumber(0);

    ta = table.getEntry("ta"); // area of target visible
    tx = table.getEntry("tx"); // horizontal offset
    ty = table.getEntry("ty"); // vertical offset
    tv = table.getEntry("tv"); // valid target? (0 or 1)

    a = ta.getDouble(0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightTargets", v);

    updateVisionState();
    updateSmartDashboard();
    //SmartDashboard.putString("testExecute", "TestExecute");
    // This method will be called once per scheduler run
  }

  public VisionState updateVisionState(){
    if(v == 1) {
      return VisionState.TARGET_VISIBLE;
    }
    return VisionState.NO_TARGET;
  }

  public double getDistance() {
    if (table == null) {
      return -1.0;
    }

    return (Constants.Vision.H2 - Constants.Vision.H1) / Math.tan(Math.toRadians(Constants.Vision.A1 + y));
  }

  public double getRawAngle() {
    return -x;
    // return 15;
  }

  public ShooterZone getShooterStateFromDistance() {
    double distance = getDistance();
    if(distance == 0.0) {
      return ShooterZone.ZONE_2;
    }
    if (distance < Constants.Vision.Distance.ZONE_2) {
      return ShooterZone.ZONE_2;
    } else if (distance < Constants.Vision.Distance.ZONE_3) {
      return ShooterZone.ZONE_3;
    } else if (distance < Constants.Vision.Distance.ZONE_4) {
      return ShooterZone.ZONE_4;
    } else if (distance < Constants.Vision.Distance.ZONE_5) {
      return ShooterZone.ZONE_5;
    } else if (distance < Constants.Vision.Distance.ZONE_6) {
      return ShooterZone.ZONE_6;
    } else
    return ShooterZone.ZONE_6;
  }

  private void updateSmartDashboard() {
    double temp_x = Math.abs(x);
    if(a >= Constants.Vision.AREA_VISIBLE) {
      if (temp_x <= Constants.Vision.Angle.ON_TARGET_X) {
        SmartDashboard.putString("angle", "on target");
      } else {
        SmartDashboard.putString("angle", "visible");
      }
    } else {
      SmartDashboard.putString("angle", "off");
    }
  } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
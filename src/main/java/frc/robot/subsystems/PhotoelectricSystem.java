// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ColorSensorSystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.AnalogInput;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotoelectricSystem extends SubsystemBase {

  private static AnalogInput analog = new AnalogInput(Constants.Photoelectric.SENSOR);
  private static int lineval = 500; 
  private static int thresh = 100;

  public PhotoelectricSystem() { //init
  }

  public static enum PhotoelectricState {
    LINE, NOT_LINE
  }

  public PhotoelectricState updateVisionState(){
    int a = analog.getValue();
    SmartDashboard.putNumber("photoelectric", a);
    if(a>=lineval-thresh &&  a<=lineval+thresh){
      return PhotoelectricState.LINE;
    }
    return PhotoelectricState.NOT_LINE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

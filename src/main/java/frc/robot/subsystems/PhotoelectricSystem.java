// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotoelectricSystem extends SubsystemBase {
  private static PhotoelectricSystem m_instance = null;

  //private static AnalogInput analog = new AnalogInput(Constants.Photoelectric.SENSOR);
  private static DigitalInput digLeft;
  private static DigitalInput digRight;

  // private static int lineval = 500; 
  // private static int thresh = 100;

  private PhotoelectricSystem() { //init
    digLeft = new DigitalInput(Constants.Photoelectric.SENSOR_LEFT);
    digRight = new DigitalInput(Constants.Photoelectric.SENSOR_RIGHT);
  }

  public static enum PhotoelectricState {
    LINE, NOT_LINE
  }

  public PhotoelectricState updatePhotoStateLeft(){
    // SmartDashboard.putNumber("PhotoTest: ", 1);
    //int a = analog.getValue();
    boolean d = digLeft.get();
    //SmartDashboard.putNumber("photoelectric", a);
    // SmartDashboard.putBoolean("d", d);
    if(!d){
      // SmartDashboard.putBoolean("Over shadow line: ", true);
      return PhotoelectricState.LINE;
    }
    // SmartDashboard.putBoolean("Over shadow line: ", false);
    return PhotoelectricState.NOT_LINE;
  }

  public PhotoelectricState updatePhotoStateRight(){
    //SmartDashboard.putNumber("PhotoTest: ", 1);
    boolean d = digRight.get();
    // SmartDashboard.putBoolean("d", d);
    if(!d){
      // SmartDashboard.putBoolean("Over shadow line: ", true);
      return PhotoelectricState.LINE;
    }
    // SmartDashboard.putBoolean("Over shadow line: ", false);
    return PhotoelectricState.NOT_LINE;
  }

  public static PhotoelectricSystem getInstance() {
    if (m_instance == null) {
      m_instance = new PhotoelectricSystem();
    }

    return m_instance;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Test: ", 1);
    updatePhotoStateLeft();
    updatePhotoStateRight();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

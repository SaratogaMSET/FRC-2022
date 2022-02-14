// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ColorSensorSystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;

import java.lang.Math;

public class ColorSensorSystem extends SubsystemBase {
  private final ColorSensorV3 colorSensor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard; //Constants.Colorwheel.COLOR_SENSOR; 

  //color sensor constants
  private Color currentColor;
  private double confidence;

  // TODO: change/tune RGB values from 0 0 0
  private final double LINE_R = 0, LINE_G = 0, LINE_B = 0;
  private final Color BLACK_LINE = new Color(LINE_R, LINE_G, LINE_B);
  // Maybe remove BLACK_LINE entirely? Since we only use its raw RGB values

  public ColorSensorSystem() { //init
    colorSensor = new ColorSensorV3(i2cPort);
  }

  private void compareColor(){
    // Read current color
    currentColor = colorSensor.getColor();
    confidence = 0.95; // Default confidence: 0.95

    // TODO: do something with this isApproxBlack
    isApproxBlack(currentColor, confidence);
  }

  private boolean isApproxBlack(Color c, double conf) {
    double rc = c.red, gc = c.green, bc = c.blue; // RGB of Color c
    double drsq = 0, dgsq = 0, dbsq = 0;
    double RGBDiff = 0;

    drsq = Math.pow(LINE_R - rc, 2);
    dgsq = Math.pow(LINE_G - gc, 2);
    dbsq = Math.pow(LINE_B - bc, 2);
    RGBDiff = Math.sqrt(drsq + dgsq + dbsq);
    
    return (RGBDiff <= conf);
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

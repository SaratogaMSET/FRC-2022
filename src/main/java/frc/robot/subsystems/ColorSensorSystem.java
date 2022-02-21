// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ColorSensorSystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;

import java.lang.Math;

public class ColorSensorSystem extends SubsystemBase {
  private final ColorSensorV3 colorSensor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard; //Constants.Colorwheel.COLOR_SENSOR; 

  //color sensor constants
  private Color currentColor;
  private double confidence;

  // TODO: tune RGB values
  private final double LINE_R = 200.0, LINE_G = 200.0, LINE_B = 200.0;

  public ColorSensorSystem() { //init
    colorSensor = new ColorSensorV3(i2cPort);
  }

  private void compareColor(){
    confidence = 500;

    // TODO: do something with this isApproxBlack
    if(isApproxBlack(confidence)) {
      SmartDashboard.putString("Color", "Black");
    } else {
      SmartDashboard.putString("Color", "Not black");
    }

    // Output raw RGB values from color sensor onto SmartDashboard
    int currColorR = colorSensor.getRed(), currColorG = colorSensor.getGreen(), currColorB = colorSensor.getBlue();
    SmartDashboard.putNumber("Raw r: ", currColorR);
    SmartDashboard.putNumber("Raw g: ", currColorG);
    SmartDashboard.putNumber("Raw b: ", currColorB);
  }

  private boolean isApproxBlack(double conf) {
    double rc = (double) colorSensor.getRed(), gc = (double) colorSensor.getGreen(), bc = (double) colorSensor.getBlue();
    double drsq = 0, dgsq = 0, dbsq = 0;
    double RGBDiff = 0;

    drsq = (LINE_R - rc) * (LINE_R - rc);
    dgsq = (LINE_G - gc) * (LINE_G - gc);
    dbsq = (LINE_B - bc) * (LINE_B - bc);
    RGBDiff = Math.sqrt(drsq + dgsq + dbsq);

    SmartDashboard.putNumber("RGBDiff: ", RGBDiff);

    return (RGBDiff <= conf);
  }

  @Override
  public void periodic() {
    compareColor();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

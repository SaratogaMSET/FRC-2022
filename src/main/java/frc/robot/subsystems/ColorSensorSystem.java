// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ColorSensorSystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;

import java.lang.Math;
import java.lang.ProcessBuilder.Redirect;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.CloseAction;

public class ColorSensorSystem extends SubsystemBase {
  private final ColorSensorV3 colorSensor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard; //Constants.Colorwheel.COLOR_SENSOR; 

  //color sensor constants
  private double confidence;
  private double conf_percent;
  private final int NB_COLORS = 6;
  private final int NB_COL_PERCENT = 3;
  private final int NB_COL_COMPONENTS = 3;
  private final String[] COL_STRS = new String[]{"Black", "Red", "Blue", "Pink", "Yellow", "Green"};
  private final int BLACK = 0,
                    RED = 1,
                    BLUE = 2,
                    PINK = 3,
                    YELLOW = 4,
                    GREEN = 5;
  private final int RED_COMPONENT = 0,
                    GREEN_COMPONENT = 1,
                    BLUE_COMPONENT = 2;
  private final int DIST_THRESHOLD = 5500;

  /* RGB values */
  // RGB values tuned assuming sensor height = 2 in.
  // Dist to bar: ~0
  // Dist to floor: 1-2 in. (?)
  // TODO: check if carpet gets flagged as black and tune black RGB values accordingly
  private final double[][] COL_RGB = new double[][]{
    {233.0, 355.0, 164.0}, // Black line RGB
    {444.0, 601.0, 269.0}, // Red paint RGB
    {646.0, 970.0, 480.0}, // Blue paint RGB
    {677.0, 931.0, 433.0}, // Pink paint RGB
    {590.0, 810.0, 260.0}, // Yellow paint RGB
    {473.0, 717.0, 270.0}, // Green paint RGB
  };

  private final double[][] COL_RGB_PERCENT = new double[][]{
    {0.473, 0.365, 0.161}, // Red
    {0.18, 0.42, 0.4}, // Blue
    {0.3, 0.5, 0.2}, // Yellow
  };

  private final double  RED_CONST = 0.41,
                        BLUE_CONST = 0.38,
                        YELLOW_CONST = 0.5,
                        GREEN_B_CONST = 0.13;

  public ColorSensorSystem() { //init
    colorSensor = new ColorSensorV3(i2cPort);
  }

  private void compareColor(){
    confidence = 87.0;
    conf_percent = 0.1;

    isApproxBlack(confidence);
    colorString(confidence, conf_percent);

    // Output raw RGB values from color sensor onto SmartDashboard
    int currColorR = colorSensor.getRed(), 
        currColorG = colorSensor.getGreen(), 
        currColorB = colorSensor.getBlue();
    SmartDashboard.putNumber("Raw r: ", currColorR);
    SmartDashboard.putNumber("Raw g: ", currColorG);
    SmartDashboard.putNumber("Raw b: ", currColorB);

    double  percentR = colorSensor.getColor().red,
            percentG = colorSensor.getColor().green,
            percentB = colorSensor.getColor().blue;
    SmartDashboard.putNumber("Percent r: ", percentR);
    SmartDashboard.putNumber("Percent g: ", percentG);
    SmartDashboard.putNumber("Percent b: ", percentB);
  }

  private boolean isApproxBlack(double conf) {
    double rc = (double) colorSensor.getRed(), gc = (double) colorSensor.getGreen(), bc = (double) colorSensor.getBlue();
    double drsq = 0.0, dgsq = 0.0, dbsq = 0.0;
    double RGBDiff = 0.0;

    drsq = (COL_RGB[BLACK][RED_COMPONENT] - rc) * (COL_RGB[BLACK][RED_COMPONENT] - rc);
    dgsq = (COL_RGB[BLACK][GREEN_COMPONENT] - gc) * (COL_RGB[BLACK][GREEN_COMPONENT] - gc);
    dbsq = (COL_RGB[BLACK][BLUE_COMPONENT] - bc) * (COL_RGB[BLACK][BLUE_COMPONENT] - bc);
    RGBDiff = Math.sqrt(drsq + dgsq + dbsq);

    SmartDashboard.putNumber("RGBDiff: ", RGBDiff);

    return (RGBDiff <= conf);
  }

  private String colorString(double conf, double conf_percent) {    
    double[] sensorRGB = new double[]{
      (double) colorSensor.getRed(),
      (double) colorSensor.getGreen(),
      (double) colorSensor.getBlue()
    };

    int i, j;
    if (sensorRGB[0] + sensorRGB[1] + sensorRGB[2] <= DIST_THRESHOLD) {

      double[][] colSqs = new double[NB_COLORS][NB_COL_COMPONENTS];
      double[] rgbDiffs = new double[NB_COLORS];

      // Raw RGB values to SmartDashboard
      SmartDashboard.putNumber("Raw r: ", sensorRGB[RED_COMPONENT]);
      SmartDashboard.putNumber("Raw g: ", sensorRGB[GREEN_COMPONENT]);
      SmartDashboard.putNumber("Raw b: ", sensorRGB[BLUE_COMPONENT]);

      /* Calculate rgbDiffs */
      for (i = BLACK; i < NB_COLORS; ++i) {

        for (j = RED_COMPONENT; j < NB_COL_COMPONENTS; ++j) {

          colSqs[i][j] = (COL_RGB[i][j] - sensorRGB[j]) * (COL_RGB[i][j] - sensorRGB[j]);

        }

        rgbDiffs[i] = Math.sqrt(colSqs[i][RED_COMPONENT] + colSqs[i][GREEN_COMPONENT] + colSqs[i][BLUE_COMPONENT]);
      
      }

      /* Color matching checks */
      for (i = BLACK; i < NB_COLORS; ++i) {

        if (rgbDiffs[i] <= conf) {
          SmartDashboard.putString("Color: ", COL_STRS[i]);
          return COL_STRS[i];
        }

      }

    } else { // Percentage checks

      double[] sensorRGBPercent = new double[]{
        colorSensor.getColor().red,
        colorSensor.getColor().green,
        colorSensor.getColor().blue,
      };      
      
      /* {0.473, 0.365, 0.161}, // Red
      {0.18, 0.42, 0.4}, // Blue
      {0.3, 0.5, 0.2}, // Yellow */

      /* Color matching checks for percentages */
      /* if ((sensorRGBPercent[GREEN_COMPONENT] >= COL_RGB_PERCENT[YELLOW][GREEN_COMPONENT]) && (sensorRGBPercent[BLUE_COMPONENT] <= COL_RGB_PERCENT[YELLOW][BLUE_COMPONENT])) {
        SmartDashboard.putString("Color: ", COL_STRS[YELLOW]);
        return COL_STRS[YELLOW];
      } else  */if (sensorRGBPercent[RED_COMPONENT] >= RED_CONST) {
        SmartDashboard.putString("Color: ", COL_STRS[RED]);
        return COL_STRS[RED];
      } else if (sensorRGBPercent[GREEN_COMPONENT] >= YELLOW_CONST) { // Lime green registers as yellow - TODO: fix
        SmartDashboard.putString("Color: ", COL_STRS[YELLOW]);
        return COL_STRS[YELLOW];
      } else if (sensorRGBPercent[BLUE_COMPONENT] >= BLUE_CONST) {
        SmartDashboard.putString("Color: ", COL_STRS[BLUE]);
        return COL_STRS[BLUE];
      } else {
        SmartDashboard.putString("Color: ", "Unknown");
        return "Unknown";
      }

    }

    SmartDashboard.putString("Color: ", "Unknown");
    return "Unknown";
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

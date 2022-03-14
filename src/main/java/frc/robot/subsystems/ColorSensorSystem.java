// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;


// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.ColorSensorSystem;
// import frc.robot.subsystems.Multi2c;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorSensorV3.RawColor;
// import edu.wpi.first.wpilibj.util.Color;

// import java.lang.Math;

// /*
// TODO:
// - Multiple color sensors: array of color sensors?
// - Read + eval multiple color sensors
// - Implement needed methods for reading from multiple color sensors (getColor, getR/G/B, etc.)
// - To read multiple values: iterate thru breakoutChannels?
// - TEST W/ I2C
// */

// public class ColorSensorSystem extends SubsystemBase {
//   //private final ColorSensorV3 colorSensor;

//   //private final I2C.Port i2cPort = I2C.Port.kOnboard; //Constants.Colorwheel.COLOR_SENSOR; 

//   //color sensor constants
//   private double confidence;
//   private double conf_percent;
//   private final int NB_COLORS = 6;
//   private final int NB_COL_COMPONENTS = 3;
//   private final String[] COL_STRS = new String[]{"Black", "Red", "Blue", "Pink", "Yellow", "Green"};
//   private final int BLACK = 0,
//                     RED = 1,
//                     BLUE = 2,
//                     //PINK = 3,
//                     YELLOW = 4;
//                     //GREEN = 5;
//   private final int RED_COMPONENT = 0,
//                     GREEN_COMPONENT = 1,
//                     BLUE_COMPONENT = 2;
//   private final int DIST_THRESHOLD = 5500;

//   /* RGB values */
//   // RGB values tuned assuming sensor height = 2 in.
//   // Dist to bar: ~0
//   // Dist to floor: 1-2 in. (?)
//   // TODO: check if carpet gets flagged as black and tune black RGB values accordingly
//   private final double[][] COL_RGB = new double[][]{
//     {233.0, 355.0, 164.0}, // Black line RGB
//     {444.0, 601.0, 269.0}, // Red paint RGB
//     {646.0, 970.0, 480.0}, // Blue paint RGB
//     {677.0, 931.0, 433.0}, // Pink paint RGB
//     {590.0, 810.0, 260.0}, // Yellow paint RGB
//     {473.0, 717.0, 270.0}, // Green paint RGB
//   };

//   private final double  RED_CONST = 0.41,
//                         BLUE_CONST = 0.38,
//                         YELLOW_CONST = 0.5;

//   /* MultI2C */
//   private Multi2c breakout;
//   private byte breakoutChannel;
//   private I2C.Port i2cPort = I2C.Port.kOnboard;
//   private ColorSensorV3 colorSensor;

//   public ColorSensorSystem() { //init
//     colorSensor = new ColorSensorV3(i2cPort);
//   }

//   public ColorSensorSystem(I2C.Port port, Multi2c breakout, byte breakoutChannel) {
//     i2cPort = port;
//     this.breakout = breakout;
//     this.breakoutChannel = breakoutChannel;

//     setActive();
//     colorSensor = new ColorSensorV3(port);
//   }

//   public void setActive() {
//     /* byte[] bChannel = {(byte)(1 << breakoutChannel)};
//     breakout.setEnabledBuses(bChannel); */
//     int[] channel = {breakoutChannel};
//     breakout.setEnabledBuses(channel); // FIXME
//   }

//   public RawColor getRawColor() {
//     setActive();
//     return colorSensor.getRawColor();
//   }

//   public Color getColor() {
//     setActive();
//     return colorSensor.getColor();
//   }

//   public int getRed() {
//     setActive();
//     return colorSensor.getRed();
//   }

//   public int getGreen() {
//     setActive();
//     return colorSensor.getGreen();
//   }

//   public int getBlue() {
//     setActive();
//     return colorSensor.getBlue();
//   }

//   private void compareColor(){
//     confidence = 87.0;
//     conf_percent = 0.1;

//     colorString(confidence, conf_percent);

//     // Output raw RGB values from color sensor(s) onto SmartDashboard
//     int currColorR = getRed(), 
//         currColorG = getGreen(), 
//         currColorB = getBlue();
//     SmartDashboard.putNumber("Raw r: ", currColorR);
//     SmartDashboard.putNumber("Raw g: ", currColorG);
//     SmartDashboard.putNumber("Raw b: ", currColorB);

//     double  percentR = getColor().red,
//             percentG = getColor().green,
//             percentB = getColor().blue;
//     SmartDashboard.putNumber("Percent r: ", percentR);
//     SmartDashboard.putNumber("Percent g: ", percentG);
//     SmartDashboard.putNumber("Percent b: ", percentB);
//   }

//   private String colorString(double conf, double conf_percent) {    
//     double[] sensorRGB = new double[]{
//       (double) getRed(),
//       (double) getGreen(),
//       (double) getBlue()
//     };

//     int i, j;
//     if (sensorRGB[0] + sensorRGB[1] + sensorRGB[2] <= DIST_THRESHOLD) {

//       double[][] colSqs = new double[NB_COLORS][NB_COL_COMPONENTS];
//       double[] rgbDiffs = new double[NB_COLORS];

//       // Raw RGB values to SmartDashboard
//       SmartDashboard.putNumber("Raw r: ", sensorRGB[RED_COMPONENT]);
//       SmartDashboard.putNumber("Raw g: ", sensorRGB[GREEN_COMPONENT]);
//       SmartDashboard.putNumber("Raw b: ", sensorRGB[BLUE_COMPONENT]);

//       /* Calculate rgbDiffs */
//       for (i = BLACK; i < NB_COLORS; ++i) {
//         for (j = RED_COMPONENT; j < NB_COL_COMPONENTS; ++j) {
//           colSqs[i][j] = (COL_RGB[i][j] - sensorRGB[j]) * (COL_RGB[i][j] - sensorRGB[j]);
//         }
//         rgbDiffs[i] = Math.sqrt(colSqs[i][RED_COMPONENT] + colSqs[i][GREEN_COMPONENT] + colSqs[i][BLUE_COMPONENT]);
//       }

//       /* Color matching checks */
//       for (i = BLACK; i < NB_COLORS; ++i) {
//         if (rgbDiffs[i] <= conf) {
//           SmartDashboard.putString("Color: ", COL_STRS[i]);
//           return COL_STRS[i];
//         }
//       }

//     } else { // Percentage checks

//       double[] sensorRGBPercent = new double[]{
//         getColor().red,
//         getColor().green,
//         getColor().blue,
//       };      
      
//       if (sensorRGBPercent[RED_COMPONENT] >= RED_CONST) {
//         SmartDashboard.putString("Color: ", COL_STRS[RED]);
//         return COL_STRS[RED];
//       } else if (sensorRGBPercent[GREEN_COMPONENT] >= YELLOW_CONST) { // Lime-green registers as yellow - TODO: fix
//         SmartDashboard.putString("Color: ", COL_STRS[YELLOW]);
//         return COL_STRS[YELLOW];
//       } else if (sensorRGBPercent[BLUE_COMPONENT] >= BLUE_CONST) {
//         SmartDashboard.putString("Color: ", COL_STRS[BLUE]);
//         return COL_STRS[BLUE];
//       } else {
//         SmartDashboard.putString("Color: ", "Unknown");
//         return "Unknown";
//       }

//     }

//     SmartDashboard.putString("Color: ", "Unknown");
//     return "Unknown";
//   }

//   @Override
//   public void periodic() {
//     compareColor();
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }

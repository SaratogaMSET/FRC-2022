// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.VisionSubsystem.VisionState;

public class LEDSubsystem extends SubsystemBase {

  private static Spark m_blinkin1;
  private static Spark m_blinkin2;

  private static double sparkValue = 0;
  // private static boolean blink = false;

  public LEDSubsystem() {

    m_blinkin1 = new Spark(0);
    m_blinkin2 = new Spark(1);

  }

  public void setStatus(boolean intakeIRGate, boolean shooterIRGate, VisionState visionState){
    if(intakeIRGate == false && shooterIRGate == false){
      sparkValue = 0.85; // BLUE
    }
    if(intakeIRGate == true && shooterIRGate == false){
      sparkValue = 0.65; // YELLOW
    }
    if(intakeIRGate == false && shooterIRGate == true){
      sparkValue = 0.65; // YELLOW
    }
    if(intakeIRGate == true && shooterIRGate == true){
      sparkValue = 0.61; // RED
    }

    // if(visionState == VisionState.TARGET_VISIBLE){
    //   sparkValue = -0.83; // -0.55, -0.97
    // }
  }

  @Override
  public void periodic() {
    m_blinkin1.set(sparkValue);
    m_blinkin2.set(sparkValue);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

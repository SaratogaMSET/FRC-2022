// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;

import java.lang.Math;

public class LEDSubsystem extends SubsystemBase {

  private static Spark m_blinkin1;
  private static Spark m_blinkin2;
  

  // private final AddressableLED m_led1;
  // private final AddressableLEDBuffer m_ledBuffer1;

  // private final AddressableLED m_led2;
  // private final AddressableLEDBuffer m_ledBuffer2;

  public LEDSubsystem() {

    m_blinkin1 = new Spark(0);
    m_blinkin2 = new Spark(1);

    // m_led1 = new AddressableLED(0);
    // m_ledBuffer1 = new AddressableLEDBuffer(15);
    // m_led1.setLength(m_ledBuffer1.getLength());

    // m_led2 = new AddressableLED(1);
    // m_ledBuffer2 = new AddressableLEDBuffer(15);
    // m_led2.setLength(m_ledBuffer2.getLength());
  }

  public void setStatus(boolean intakeIRGate, boolean shooterIRGate){
    if(intakeIRGate == false && shooterIRGate == false){
      setGreen();
    }
    if(intakeIRGate == true && shooterIRGate == false){
      setYellow();
    }
    if(intakeIRGate == false && shooterIRGate == true){
      setYellow();
    }
    if(intakeIRGate == true && shooterIRGate == true){
      setRed();
    }
  }

  private void setGreen() {
    m_blinkin1.set(0.77);
    m_blinkin2.set(0.77);
  }
  private void setYellow() {
    m_blinkin1.set(0.65);
    m_blinkin2.set(0.65);
  }
  private void setRed() {
    m_blinkin1.set(0.61);
    m_blinkin2.set(0.61);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Feeder extends SubsystemBase {
  TalonSRX topMotor;
  TalonSRX bottomMotor;

  public static enum FeederState {
    TEST,
    RUN
  }

  public Feeder() {
    topMotor = new TalonSRX(Constants.FeederConstants.TOP_MOTOR);
    bottomMotor = new TalonSRX(Constants.FeederConstants.BOTTOM_MOTOR);
  }

  public void setTopMotor(double velocity) {
    topMotor.set(ControlMode.PercentOutput, velocity);
  }

  public void setBottomMotor(double velocity) {
    bottomMotor.set(ControlMode.PercentOutput, velocity);
  }

  public void diagnostics() {
    String topStatus = "Feeder Top Status";
    String bottomStatus = "Feeder Bottom Status";

    try {
      setTopMotor(-0.1);
      if (topMotor.getMotorOutputPercent() > -0.08 || topMotor.getMotorOutputPercent() < -0.12) {
        SmartDashboard.putString(topStatus, "Failed");
      } else
        SmartDashboard.putString(topStatus, "Success");
    } catch (Exception e) {
      SmartDashboard.putString(topStatus, "Failed");
    }

    try {
      setBottomMotor(0.1);
      if (bottomMotor.getMotorOutputPercent() < 0.08 || bottomMotor.getMotorOutputPercent() > 0.12) {
        SmartDashboard.putString(bottomStatus, "Failed");
      } else
        SmartDashboard.putString(bottomStatus, "Success");
    } catch (Exception e) {
      SmartDashboard.putString(bottomStatus, "Failed");
    }
  }

  @Override
  public void periodic() {
  }
}

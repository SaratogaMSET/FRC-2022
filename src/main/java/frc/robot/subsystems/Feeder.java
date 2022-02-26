// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Feeder extends SubsystemBase {
  private TalonSRX topMotor;
  private TalonSRX bottomMotor;
  private DigitalInput shooterGate;
  private DigitalInput intakeGate;
  public boolean inIntakeFeeder;
  public boolean inShooterFeeder;
  private boolean runIntakeFeeder;
  private boolean runShooterFeeder;

  public static enum FeederState {
    TEST,
    RUN
  }

  public Feeder() {
    topMotor = new TalonSRX(20);
    bottomMotor = new TalonSRX(22);
    shooterGate = new DigitalInput(0);
    intakeGate = new DigitalInput(1);
    inIntakeFeeder = false;
    inShooterFeeder = false;
    runIntakeFeeder = false;
    runShooterFeeder = false;
  }

  public void updateGates() {
    inIntakeFeeder = !intakeGate.get();
    inShooterFeeder = !shooterGate.get();
    SmartDashboard.putBoolean("Ball in Intake", inIntakeFeeder);
    SmartDashboard.putBoolean("Ball in Shooter", inShooterFeeder);
    if (inIntakeFeeder && inShooterFeeder) {
      runIntakeFeeder = false;
      runShooterFeeder = false;
    } else if (inIntakeFeeder) {
      runIntakeFeeder = true;
      runShooterFeeder = true;
    } else if (inShooterFeeder) {
      runIntakeFeeder = true;
      runShooterFeeder = false;
    } else {
      runIntakeFeeder = true;
      runShooterFeeder = true;
    }
  }

  public void setTopMotor(double velocity) {
    if (runShooterFeeder)
      topMotor.set(ControlMode.PercentOutput, velocity);
    else
      topMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setBottomMotor(double velocity) {
    if (runIntakeFeeder)
      bottomMotor.set(ControlMode.PercentOutput, velocity);
    else
      bottomMotor.set(ControlMode.PercentOutput, 0.0);
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

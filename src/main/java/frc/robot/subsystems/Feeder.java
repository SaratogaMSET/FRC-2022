// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Feeder extends SubsystemBase {
  private TalonFX shooterFeederMotor;
  private TalonFX intakeFeederMotor;
  private DigitalInput shooterGate;
  private DigitalInput intakeGate;
  public boolean inIntakeFeeder;
  public boolean inShooterFeeder;
  private boolean runIntakeFeeder;
  private boolean runShooterFeeder;
  private double shooterFeederVelocity;
  private double intakeFeederVelocity;

  public static enum FeederState {
    TEST,
    INTAKE,
    OUTTAKE,
    IDLE
  }

  public Feeder() {
    shooterFeederMotor = new TalonFX(Constants.FeederConstants.SHOOTER_FEEDER_MOTOR);
    intakeFeederMotor = new TalonFX(Constants.FeederConstants.INTAKE_FEEDER_MOTOR);
    shooterGate = new DigitalInput(Constants.FeederConstants.IR_GATES[0]);
    intakeGate = new DigitalInput(Constants.FeederConstants.IR_GATES[1]);
    inIntakeFeeder = false;
    inShooterFeeder = false;
    runIntakeFeeder = false;
    runShooterFeeder = false;
    intakeFeederMotor.setInverted(true);
  }

  public void updateGates() {
    inIntakeFeeder = !intakeGate.get();
    inShooterFeeder = !shooterGate.get();
    SmartDashboard.putBoolean("Ball in Intake", inIntakeFeeder);
    SmartDashboard.putBoolean("Ball in Shooter", inShooterFeeder);
    if (shooterFeederVelocity < 0.0 || intakeFeederVelocity < 0.0) {
      runIntakeFeeder = true;
      runShooterFeeder = true;
    } else if (inIntakeFeeder && inShooterFeeder) {
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

  public FeederState getState() {
    double measuredShooterFeeder = shooterFeederMotor.getMotorOutputPercent();
    double measuredIntakeFeeder = intakeFeederMotor.getMotorOutputPercent();
    if (measuredShooterFeeder > 0.0 || measuredIntakeFeeder > 0.0)
      return FeederState.INTAKE;
    else if (measuredShooterFeeder < 0.0 || measuredIntakeFeeder < 0.0)
      return FeederState.OUTTAKE;
    else
      return FeederState.IDLE;
  }

  public void setShooterFeeder(double velocity) {
    shooterFeederVelocity = velocity;
    updateGates();
    if (runShooterFeeder)
      shooterFeederMotor.set(ControlMode.PercentOutput, velocity);
    else
      shooterFeederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setIntakeFeeder(double velocity) {
    intakeFeederVelocity = velocity;
    updateGates();
    if (runIntakeFeeder)
      intakeFeederMotor.set(ControlMode.PercentOutput, velocity);
    else
      intakeFeederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void diagnostics() {
    String shooterStatus = "Feeder Shooter Status";
    String feederStatus = "Feeder Feeder Status";
    String feederGate = "Feeder Gate Status";
    String shooterGate = "Shooter Gate Status";

    try {
      setShooterFeeder(0.1);
      if (shooterFeederMotor.getMotorOutputPercent() > -0.08 || shooterFeederMotor.getMotorOutputPercent() < -0.12) {
        SmartDashboard.putString(shooterStatus, "Failed");
      } else
        SmartDashboard.putString(shooterStatus, "Success");
    } catch (Exception e) {
      SmartDashboard.putString(shooterStatus, "Failed");
    }

    try {
      setIntakeFeeder(0.1);
      if (intakeFeederMotor.getMotorOutputPercent() < 0.08 || intakeFeederMotor.getMotorOutputPercent() > 0.12) {
        SmartDashboard.putString(feederStatus, "Failed");
      } else
        SmartDashboard.putString(feederStatus, "Success");
    } catch (Exception e) {
      SmartDashboard.putString(feederStatus, "Failed");
    }

    try {
      updateGates();
      if(!inIntakeFeeder)
        SmartDashboard.putString(feederGate, "Success");
      else
        SmartDashboard.putString(feederGate, "Failed");
    } catch (Exception e){
      SmartDashboard.putString(feederStatus, "Failed");
    }

    try {
      updateGates();
      if(!inShooterFeeder)
        SmartDashboard.putString(shooterGate, "Success");
      else
        SmartDashboard.putString(shooterGate, "Failed");
    } catch (Exception e){
      SmartDashboard.putString(shooterGate, "Failed");
    }
  }

  @Override
  public void periodic() {
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  public TalonFX shooterFeederMotor;
  public TalonFX intakeFeederMotor;

  public DigitalInput shooterGate;
  public DigitalInput intakeGate;
  
  public static enum FeederState {
    INTAKE,
    OUTTAKE,
    IDLE
  }

  public FeederSubsystem() {
    shooterFeederMotor = new TalonFX(Constants.FeederConstants.SHOOTER_FEEDER_MOTOR);
    intakeFeederMotor = new TalonFX(Constants.FeederConstants.INTAKE_FEEDER_MOTOR);
    intakeFeederMotor.setInverted(true);

    shooterGate = new DigitalInput(Constants.FeederConstants.IR_GATES[0]);
    intakeGate = new DigitalInput(Constants.FeederConstants.IR_GATES[1]);
  }

  public void stopFeeder() {
    setIntakeFeeder(0.0);
    setShooterFeeder(0.0);
  }

  private void setIntakeFeeder(double velocity) {
    if (velocity == 0.0) {
      intakeFeederMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      if (canRunIntakeFeeder()) {
        intakeFeederMotor.set(ControlMode.PercentOutput, velocity);
      } else {
        intakeFeederMotor.set(ControlMode.PercentOutput, 0.0);
      }
    }
  }

  private void setShooterFeeder(double velocity) {
    if (velocity == 0.0) {
      shooterFeederMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      if (canRunShooterFeeder()) {
        shooterFeederMotor.set(ControlMode.PercentOutput, velocity);
      } else {
        shooterFeederMotor.set(ControlMode.PercentOutput, 0.0);
      }
    }
  }

  private boolean canRunIntakeFeeder() {
    return intakeGate.get();
  }

  private boolean canRunShooterFeeder() {
    return shooterGate.get();
  }

  public void runIntakeIfPossible(double shooterFeederSpeed, double intakeFeederSpeed) {
    setShooterFeeder(shooterFeederSpeed);
    setIntakeFeeder(intakeFeederSpeed);
  }

  public void runOuttake(double shooterFeederSpeed, double intakeFeederSpeed) {
    shooterFeederMotor.set(ControlMode.PercentOutput, -shooterFeederSpeed);
    intakeFeederMotor.set(ControlMode.PercentOutput, -intakeFeederSpeed);
  }

  public FeederState getFeederState() {
    double measuredShooterFeeder = shooterFeederMotor.getMotorOutputPercent();
    double measuredIntakeFeeder = intakeFeederMotor.getMotorOutputPercent();
    if (measuredShooterFeeder > 0.0 || measuredIntakeFeeder > 0.0)
      return FeederState.INTAKE;
    else if (measuredShooterFeeder < 0.0 || measuredIntakeFeeder < 0.0)
      return FeederState.OUTTAKE;
    else
      return FeederState.IDLE;
  }

  @Override
  public void periodic() {
  }
}

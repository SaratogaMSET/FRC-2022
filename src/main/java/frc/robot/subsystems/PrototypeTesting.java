// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class PrototypeTesting extends SubsystemBase {
    // private final Joystick joystick34;
    // private final Joystick joystick40;
    private final TalonSRX m_40;
    private final TalonSRX m_34;
    private double limit40;
    private double limit34;

    public PrototypeTesting() {
        m_40 = new TalonSRX(Constants.Hang.HANG_LEFT_MOTOR);
        m_34 = new TalonSRX(Constants.Hang.HANG_RIGHT_MOTOR);
    }

    public void run34(double velocityPercent) {
        limit34 = -limit34 / 2 + 0.5;
        m_34.set(ControlMode.PercentOutput, limit34 * velocityPercent);
        SmartDashboard.putNumber("Motor 34 Velocity", -limit34 * velocityPercent);
    }

    public void run40(double velocityPercent) {
        limit40 = limit40 / 2 + 0.5;
        m_40.set(ControlMode.PercentOutput, limit40 * velocityPercent);
        SmartDashboard.putNumber("Motor 40 Velocity", -limit40 * velocityPercent);
    }

    public void set34Limit(double limit) {
        limit34 = -limit / 2 + 0.5;
        SmartDashboard.putNumber("Motor 34 Limit", limit34);
    }

    public void set40Limit(double limit) {
        limit40 = -limit / 2 + 0.5;
        SmartDashboard.putNumber("Motor 40 Limit", limit40);
    }

    public void reset(){
        SmartDashboard.putData("Motor 34 Limit", null);
        SmartDashboard.putData("Motor 40 Limit", null);
    }

    @Override
    public void periodic() {
    }
}

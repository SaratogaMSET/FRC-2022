// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    // private TalonSRX motor1Falcon;
    // private TalonSRX motor2Falcon;
    private Solenoid rightValve;
    private Solenoid leftValve;
    private Solenoid valve1;
    private Solenoid valve2;
    private Solenoid valve3;
    private Solenoid valve4;
    private Solenoid valve5;
    private Solenoid valve6;

    public static enum IntakeState {
        TEST,
        RUN
    }

    public Intake() {
        // motor1 = new Talon(34);

        // motor1Falcon = new TalonSRX(Constants.IntakeConstants.RIGHT_MOTOR); // Right
        // motor
        // motor2Falcon = new TalonSRX(Constants.IntakeConstants.LEFT_MOTOR); // Left
        // motor
        // motor2Falcon.setInverted(true); // The left motor is inverted

        rightValve = new Solenoid(2, PneumaticsModuleType.REVPH, 7); // PAREMETERS: ???, foward channel, reverse channel
        leftValve = new Solenoid(2, PneumaticsModuleType.REVPH, 6); // PAREMETERS: ???, foward channel, reverse channel
        valve1 = new Solenoid(2, PneumaticsModuleType.REVPH, 0); // PAREMETERS: ???, foward channel, reverse channel
        valve2 = new Solenoid(2, PneumaticsModuleType.REVPH, 1);
        valve3 = new Solenoid(2, PneumaticsModuleType.REVPH, 2); // PAREMETERS: ???, foward channel, reverse channel
        valve4 = new Solenoid(2, PneumaticsModuleType.REVPH, 3);
        valve5 = new Solenoid(2, PneumaticsModuleType.REVPH, 4); // PAREMETERS: ???, foward channel, reverse channel
        // valve6 = new Solenoid(2, PneumaticsModuleType.REVPH, 5);
    }

    @Override
    public void periodic() {}


    public void deploy(boolean status) {
        if (status) { // moves the piston out if the status is true (intake down)
            rightValve.set(true);
            leftValve.set(true);
            valve1.set(true);
            valve2.set(true);
            valve3.set(true);
            valve4.set(true);
            valve5.set(true);
            // valve6.set(true);
        } else { // moves the piston in if the status is false (intake up)
            rightValve.set(false);
            leftValve.set(false);
            valve1.set(false);
            valve2.set(false);
            valve3.set(false);
            valve4.set(false);
            valve5.set(false);
            // valve6.set(false);
        }
    }

    public void stopAll() {
        // motor1Falcon.set(ControlMode.PercentOutput, 0);
        // motor2Falcon.set(ControlMode.PercentOutput, 0);
        rightValve.set(false);
        leftValve.set(false);
        valve1.set(false);
            valve2.set(false);
            valve3.set(false);
            valve4.set(false);
            valve5.set(false);
            // valve6.set(false);
    }

    public void diagnostics() {
        String leftPistonStatus = "Left Piston Status";
        String rightPistonStatus = "Right Piston Status";

        try {
            deploy(true);
            if (rightValve.get()) {
                SmartDashboard.putString(rightPistonStatus, "Success");
            } else
                SmartDashboard.putString(rightPistonStatus, "Failed");
        } catch (Exception e) {
            SmartDashboard.putString(rightPistonStatus, "Failed");
        }

        try {
        deploy(true);
        if (leftValve.get()) {
        SmartDashboard.putString(leftPistonStatus, "Success");
        } else
        SmartDashboard.putString(leftPistonStatus, "Failed");
        } catch (Exception e) {
        SmartDashboard.putString(leftPistonStatus, "Failed");
        }
    }
}
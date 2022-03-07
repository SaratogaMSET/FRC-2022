// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private Solenoid rightValve;
    private Solenoid leftValve;

    public static enum IntakeState {
        TEST,
        DOWN,
        UP
    }

    public IntakeSubsystem() {
        // rightValve = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.IntakeConstants.RIGHT_PISTON);
        // leftValve = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.IntakeConstants.LEFT_PISTON);
    }

    @Override
    public void periodic() {}


    public void deploy(boolean status) {
        if (status) { // moves the piston out if the status is true (intake down)
            rightValve.set(true);
            // leftValve.set(true);
        } else { // moves the piston in if the status is false (intake up)
            rightValve.set(false);
            leftValve.set(false);
        }
    }

    public IntakeState updateIntakeState(){
        // if(rightValve.get()){
        //     return IntakeState.DOWN;
        // } else {
        //     return IntakeState.UP;
        // }
        return IntakeState.UP;
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
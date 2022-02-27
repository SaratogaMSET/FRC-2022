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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

        public static enum IntakeState {
                INTAKE,
                OUTTAKE,
                FLIP_DOWN,
                FLIP_UP,
                IDLE
        }

        private Solenoid rightValve;
        private Solenoid leftValve;
        

        public Intake() {
                rightValve = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.IntakeConstants.PISTON_PORTS[0]); // PAREMETERS: ???, foward channel, reverse channel
                leftValve = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.IntakeConstants.PISTON_PORTS[1]); // PAREMETERS: ???, foward channel, reverse channel
        }

        @Override
        public void periodic() {

        }

        // ACTIONS-----------------------------------------------------------------------------------------------------------------
        
        public void deploy(boolean status) {
                if (status) { // moves the piston out if the status is true (intake down)
                        rightValve.set(true);
                        leftValve.set(true);
                } else { // moves the piston in if the status is false (intake up)
                        rightValve.set(false);
                        leftValve.set(false);
                }
        }

        public void stopAll() {
                rightValve.set(false);
                leftValve.set(false);
        }

        // GET STUFF----------------------------------------------------------------------------------------------------------------

        public IntakeState updateState() {

                if (rightValve.get() == true)
                        return IntakeState.FLIP_DOWN;
                else if (rightValve.get() == false)
                        return IntakeState.FLIP_UP;
                return IntakeState.IDLE;
        }
}

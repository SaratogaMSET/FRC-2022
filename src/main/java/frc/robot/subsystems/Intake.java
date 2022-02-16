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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Intake extends SubsystemBase {

        public static enum IntakeState {
                INTAKE,
                OUTTAKE,
                FLIP_DOWN,
                FLIP_UP, 
                IDLE //hi
        }



        // private TalonSRX motor1Falcon;
        // private TalonSRX motor2Falcon;
        private Solenoid rightValve;
        private Solenoid leftValve;
        private Compressor compressor;
        private boolean enabled;
        private boolean pressureSwitch;
        private double current;

        public Intake() {
                //motor1 = new Talon(34);

                // motor1Falcon = new TalonSRX(Constants.IntakeConstants.RIGHT_MOTOR); // Right motor
                // motor2Falcon = new TalonSRX(Constants.IntakeConstants.LEFT_MOTOR); // Left motor
                // motor2Falcon.setInverted(true); // The left motor is inverted
                compressor = new Compressor(2, PneumaticsModuleType.REVPH);
                rightValve = new Solenoid(2, PneumaticsModuleType.REVPH, 7); // PAREMETERS: ???, foward channel, reverse channel
                leftValve = new Solenoid(2, PneumaticsModuleType.REVPH, 5); // PAREMETERS: ???, foward channel, reverse channel
        }

        @Override
        public void periodic() {

        }

        public void run(double speed) {
                // compressor.enableDigital();
                // motor1Falcon.set(ControlMode.PercentOutput, speed);
                // motor2Falcon.set(ControlMode.PercentOutput, speed);
        }

        public void enableCompressor(boolean enable){
                if(enable)
                        compressor.enableDigital();
                else
                        compressor.disable();
                enabled = compressor.enabled();
                pressureSwitch = compressor.getPressureSwitchValue();
                current = compressor.getCurrent();
        }

        public void deploy(boolean status) {
                if(status) { // moves the piston out if the status is true (intake down)
                        rightValve.set(true);
                        // // leftValve.set(kForward);
                }
                else { // moves the piston in if the status is false (intake up)
                        rightValve.set(false);
                        // // leftValve.set(kReverse);
                }
        }

        public void stopAll() {
                // motor1Falcon.set(ControlMode.PercentOutput, 0);
                // motor2Falcon.set(ControlMode.PercentOutput, 0);
                rightValve.set(false);
                compressor.disable();
                // leftValve.set(kOff);
        }

        public IntakeState updateState() {
                // double rightVelocity = motor1Falcon.getMotorOutputPercent(); // 0-1
                // double leftVelocity = motor2Falcon.getMotorOutputPercent(); // 0-1
                
                // if((rightVelocity > 0) && (leftVelocity > 0)) return IntakeState.INTAKE;
                // // else if((rightVelocity < 0) && (leftVelocity < 0)) return IntakeState.OUTTAKE;
                // if (rightValve.get() == kForward) return IntakeState.FLIP_DOWN;
                // else if (rightValve.get() == kOff) return IntakeState.FLIP_UP;
                 return IntakeState.IDLE;
        }


}
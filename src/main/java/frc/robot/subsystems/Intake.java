// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
                IDLE
        }


        //private Talon motor1;
        // private TalonFX motor2;

        private WPI_TalonFX motor1Falcon;
        private WPI_TalonFX motor2Falcon;
        private DoubleSolenoid rightValve;
        private DoubleSolenoid leftValve;

        public Intake() {
                //motor1 = new Talon(34);

                motor1Falcon = new WPI_TalonFX(Constants.IntakeConstants.RIGHT_MOTOR); // Right motor
                motor2Falcon = new WPI_TalonFX(Constants.IntakeConstants.LEFT_MOTOR); // Left motor
                motor2Falcon.setInverted(true); // The left motor is inverted

                
                rightValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.FORWARD_CHANNEL[0], Constants.IntakeConstants.REVERSE_CHANNEL[0]); // PAREMETERS: ???, foward channel, reverse channel
                leftValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.FORWARD_CHANNEL[1], Constants.IntakeConstants.REVERSE_CHANNEL[1]); // PAREMETERS: ???, foward channel, reverse channel

        }

        @Override
        public void periodic() {

        }

        public void run(double speed) {
                motor1Falcon.set(speed);
                motor2Falcon.set(speed);
        }

        public void deploy(boolean status) {
                if(status) { // moves the piston out if the status is true (intake down)
                        rightValve.set(kForward);
                        leftValve.set(kForward);
                        rightValve.toggle();
                        leftValve.toggle();
                }
                else { // moves the piston in if the status is false (intake up)
                        rightValve.set(kReverse);
                        leftValve.set(kReverse);
                        rightValve.toggle();
                        leftValve.toggle();
                }
        }

        public void stopAll() {
                motor1Falcon.set(0);
                motor2Falcon.set(0);
                rightValve.set(kOff);
                leftValve.set(kOff);
        }

        public IntakeState updateState() {
                double rightVelocity = motor1Falcon.get(); // 0-1
                double leftVelocity = motor2Falcon.get(); // 0-1
                
                if((rightVelocity > 0) && (leftVelocity > 0)) return IntakeState.INTAKE;
                else if((rightVelocity < 0) && (leftVelocity < 0)) return IntakeState.OUTTAKE;
                else if (rightValve.get() == kForward) return IntakeState.FLIP_DOWN;
                else if (rightValve.get() == kReverse) return IntakeState.FLIP_UP;
                else return IntakeState.IDLE;
        }


}

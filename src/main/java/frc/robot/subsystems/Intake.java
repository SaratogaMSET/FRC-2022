// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;



public class Intake extends SubsystemBase {

        public static enum IntakeState {
                INTAKE,
                OUTTAKE,
                DOWN,
                UP, 
                IDLE
        }


        private Talon motor1;
        // private TalonFX motor2;

        private Spark motor1Neo;
        private Spark motor2Neo;
        private DoubleSolenoid rightValve;
        private DoubleSolenoid leftValve;

        public Intake() {
                motor1 = new Talon(34);

                motor1Neo = new Spark(2); // Right motor
                motor2Neo = new Spark(3); // Left motor
                motor2Neo.setInverted(true); // The left motor is inverted

                
                rightValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2); // PAREMETERS: ???, foward channel, reverse channel
                leftValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4); // PAREMETERS: ???, foward channel, reverse channel

        }

        @Override
        public void periodic() {

        }

        public void run(double speed) {
                motor1Neo.set(speed);
                motor2Neo.set(speed);
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
                motor1Neo.set(0);
                motor2Neo.set(0);
                rightValve.set(kOff);
                leftValve.set(kOff);
        }

        public IntakeState updateState() {
                double rightVelocity = motor1Neo.get(); // 0-1
                double leftVelocity = motor2Neo.get(); // 0-1
                
                if((rightVelocity > 0) && (leftVelocity > 0)) return IntakeState.INTAKE;
                else if((rightVelocity < 0) && (leftVelocity < 0)) return IntakeState.OUTTAKE;
                else if (rightValve.get() == kForward) return IntakeState.DOWN;
                else if (rightValve.get() == kReverse) return IntakeState.UP;
                else return IntakeState.IDLE;
        }


}

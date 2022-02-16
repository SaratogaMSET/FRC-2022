// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

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
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

        public static enum ShooterState {
            HIGHER,
            LOWER,
            IDLE
        }

        private Talon motor1;
        private Talon motor2;

        public Shooter() {
            motor1 = new Talon(56); // 56 is a placeholder, we need to change this

        }

        @Override
        public void periodic() {

        }

        public void run(double speed2) {
            double speed = 0.0;
            if(updateState().equals(ShooterState.HIGHER)) speed = 0.5; // REVIEW
            else if(updateState().equals(ShooterState.LOWER)) speed = 0.3; // REVIEW
            else speed = 0.0;
            motor1.set(speed);
            motor2.set(speed2);
        }

        public void stopAll() {
            motor1.set(0.0);
            motor1.set(0.0);
    }

        public ShooterState updateState() {
            double velocity = motor2.get(); //0-1
            
            if(velocity > 0.5) return ShooterState.HIGHER; // REVIEW
            else if(velocity < 0.5) return ShooterState.LOWER; // REVIEW
            else return ShooterState.IDLE; // REVIEW
        }

        
}
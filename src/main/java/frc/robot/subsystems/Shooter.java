// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;

// import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
// import com.swervedrivespecialties.swervelib.SwerveModule;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Shooter extends SubsystemBase {

//         private final LazyTalonFX shooterMotor;
//         private final LazyTalonFX lsMotor;

//         public Shooter() {
//                 shooterMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
//                 lsMotor = new TalonFX(Constants.ShooterConstants.LS_MOTOR);
//         }

//         @Override
//         public void periodic() {

//         }

//         public void run(double d) {
//                 shooterMotor.set(0.5);
//         }
// }

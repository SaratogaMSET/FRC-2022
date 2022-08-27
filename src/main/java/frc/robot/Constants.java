// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Drivetrain {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5588;
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
// Fix this portion to be correct when robot

        // public static final double driveKS = (0.667 / 12); 

        public static final double driveKV = 0; //obv fix 
        
        // public static final double driveKA = (0.27 / 12); 

// Double check the rest but the above is def wrong
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        // Current Limiting 
        public static final int angleContinuousCurrentLimit = 25; //tune
        public static final int anglePeakCurrentLimit = 40; //tune
        public static final double anglePeakCurrentDuration = 0.1; //tune
        public static final boolean angleEnableCurrentLimit = false;

        public static final int driveContinuousCurrentLimit = 35; //tune
        public static final int drivePeakCurrentLimit = 60; //tune
        public static final double drivePeakCurrentDuration = 0.1; //tune
        public static final boolean driveEnableCurrentLimit = false;
        //End of Current Limiting
        public static final double openLoopRamp = 0.25;  //check/tune
        public static final double closedLoopRamp = 0.0; //check/tune

        public static final double driveGearRatio = (6.85 / 1.0); //6.85:1

        public static final double angleGearRatio = (12.8/1.0); 
        
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = true; //CHECK
        public static final boolean invertGyro = false;
        public static final double stickDeadband = 0.05; //this may need to change later

        //-------End of the new constants----------------------------

        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;
        //referred to as number 0 in the swerve module.java
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 36;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 37;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 47;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-170.0);

        //will be referred to as number 1

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 32;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 33;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 43;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(65.0);
        //will be referred to as number 2
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 34;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 35;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 45;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(25.0+180);
        //will be referred to as number 3
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 30;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 31;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 41;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(125.0);

        public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

        public static final double maxSpeed =  (6380.0 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

        public static final double maxAngularVelocity = maxSpeed /
          Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0); //we may need to check units
        //for the Drive Motors, we will only be using the kX controllers since we only need one
        public static final double kPXController = 0.00; //tune
        public static final double kIXController = 0.000; //tune


        public static final double kPYController = 0.00; //tune
        public static final double kIYController = 0.000; //tune
        public static final double kPThetaControllerTrajectory = 0;

        public static final double kPThetaController = 0.06; //.07655
        public static final double kIThetaController = 0.052; //tune

        public static final double kPThetaAimLock = 0.1; //.08
        public static final double kIThetaAimLock = 0.002; //.052
    }
    public static class AutonConstants{
        public static final double kMaxSpeedMetersPerSecond = Drivetrain.maxSpeed;
        public static final double kMaxAccelerationMetersPerSecondSquared = Drivetrain.maxAngularVelocity;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
    public static class Vision{
        public static final int LED = 3;
        public static final double H1 = 19; // distance between limelight and ground (height of limelight mount)
        public static final double H2 = 90.75; // height of target
        public static final double A1 = 35; // angle from horizontal axis
        public static final double AREA_VISIBLE = 1; //if area is large enough to be visible (ta)
        
        public static class Distance {
            public static final int ZONE_1 = 50;
            public static final int ZONE_2 = 120;
            public static final int ZONE_3 = 160;
            public static final int ZONE_4 = 205;
            public static final int ZONE_5 = 246;
            public static final int ZONE_6 = 256;
            public static final int ZONE_7 = 279;

            public static final int LIRP_1 = 80;
            public static final int LIRP_2 = 150;
            public static final int LIRP_3 = 270;
            // public static final int ZONE_1 = 117;
            // public static final int ZONE_2 = 215;
            // public static final int ZONE_3 = 285;
            // public static final int ZONE_4 = 330;
        }
 

        public static class Angle{
            public static final double ON_TARGET_X = 1;
        }
    }

    public static class ShooterConstants {
        public static final int SHOOTER_MOTOR1 = 23;
        public static final int SHOOTER_MOTOR2 = 22; // Port should be negative speed              //CHECK+MAYBE REPLACE
  
        public static final double kFalconSensorUnitsToRPM = (600.0 / 2048.0);
        public static final double kFalcon500FreeSpeed = 6380;

        public static final int SHOOTER_SOLENOID = 1;

        public static enum DistanceConstants {
            // ZONE_1 (3509/kFalcon500FreeSpeed, 60),
            ZONE_1 (0.43, true),
            ZONE_2 (0.44, true),
            ZONE_3 (.48, true),
            ZONE_4 (.51 , true),
            ZONE_5 (0.57, true),
            ZONE_6 (0.69, true),
            ZONE_7 (0.728, true),
            EMERGENCY (0.25, false),
            TEST (0.49, true),
            
            LIRP_1 (0.45, true),
            LIRP_2 (0.49, true),
            LIRP_3 (0.68, true);

            private double percentRPM;
            private boolean hoodAngle; //final?

            DistanceConstants (double percentRPM, boolean hoodAngle) {
                this.percentRPM = percentRPM;
                this.hoodAngle = hoodAngle;
            }
             public double getPercentOutput() {
                return percentRPM;
            }
             public boolean getHoodAngle() {
                return hoodAngle;
            }
        }

        public static enum AngleConstants {
            // ZONE_1 (3509/kFalcon500FreeSpeed, 60),
            TWOFIVE (25),
            FOURZERO (40);

            private double hoodAngle; //final?
            AngleConstants (double hoodAngle) {
                this.hoodAngle = hoodAngle;
            }
             public double getAngle() {
                return hoodAngle;
            }
             public void setHoodAngle(double val){
                hoodAngle = val;
            }
        }
    }
 
    public static class HangConstants {
        public static final int HANG_RIGHT_MOTOR = 26;
        public static final int HANG_LEFT_MOTOR = 25;
        public static final int RIGHT_HANG_LIMIT_SWITCH = 4;
        public static final int LEFT_HANG_LIMIT_SWITCH = 5;

        public static final int HANG_SOLENOID = 0;

        public static final int HANG_MAX_ENCODER_COUNTS = 255000; //290000
        public static final int HANG_HALF_ENCODER_COUNTS = HANG_MAX_ENCODER_COUNTS/2;
        public static final int HANG_ENCODER_SOFT_STOP = 15000;
    }
    
    public static class Photoelectric {
        public static final int SENSOR_LEFT = 9;
        public static final int SENSOR_RIGHT = 10;
    }

    public static class OIConstants {
        public static final int JOYSTICK_DRIVE_VERTICAL = 2;
        public static final int JOYSTICK_DRIVE_HORIZONTAL = 3;
    }
    
    public static class FeederConstants {
        public static final int SHOOTER_FEEDER_MOTOR = 21;
        public static final int INTAKE_FEEDER_MOTOR = 20;
        public static final int IR_GATES[] = {0, 2};
    }

    public static class IntakeConstants {
        public static final int INTAKE_SOLENOID = 2; 
    }
    
}

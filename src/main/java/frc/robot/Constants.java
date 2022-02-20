// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

        public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 0; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 0; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 0; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back right steer offset
    }

    public static class FeederConstants{
        public static final int TOP_MOTOR = 20;
        public static final int BOTTOM_MOTOR = 21;
        public static final int IR_GATES[] = {0, 1};
    }

    public static class IntakeConstants{ // Intake Motor Ports
        public static final int LEFT_MOTOR = 10; // NOTE: SET THIS PORT NUMBER LATER
        public static final int RIGHT_MOTOR = 11;
        public static final int PISTON_PORTS[] = {6, 7}; // The forward channel on the module to control (0..7)
        //public static final int MODULE_NUMBER[] = {0, 0}; // The module number of the solenoid module to use.
    }
    
    public static class ShooterConstants{
        // public static final int SHOOTER_MOTOR = 0;
        // public static final int LS_MOTOR = 0;
    }

    public static class OIConstants {
        public static final int JOYSTICK_DRIVE_VERTICAL = 2;
        public static final int JOYSTICK_DRIVE_HORIZONTAL = 3;
        public static final int OPERATOR_JOYSTICK = 3;
        public static final int GAMEPAD = 2;
    }
















    
}
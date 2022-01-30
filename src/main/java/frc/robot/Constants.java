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
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.9144;
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6096;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 32;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 33;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 43;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(25.0); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 34;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 35;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 45;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-20.0); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 41;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(40.0); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 36;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 37;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 47;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(170.0); // FIXME Measure and set back right steer offset
    }

    public static class Vision{
        public static final int LED = 3;
        public static final double H1 = 19; // distance between limelight and ground (height of limelight mount)
        public static final double H2 = 90.75; // height of target
        public static final double A1 = 35; // angle from horizontal axis
        public static final double AREA_VISIBLE = 1; //if area is large enough to be visible (ta)
        
        public static class Distance {
            public static final int GREEN = 90; 
            public static final int YELLOW = 150;
            public static final int BLUE = 190;
            public static final int RED = 270;
        }

        public static class Angle{
            // diff zones
            // limelight 2
            // x (all in plus minus)
            public static final double ON_TARGET_X = 1;
            /*
            public static final double CLOSE_X = 6;
            public static final double MEDIUM_X = 12;
            public static final double OFF_X = 29.8; 
            */
            // y (all in plus minus)
            /*
            public static final double ON_TARGET_Y = 3;
            public static final double CLOSE_Y = 6; 
            public static final double MEDIUM_Y = 12;
            public static final double OFF_Y = 24.85; 
            */
        }
    }
}

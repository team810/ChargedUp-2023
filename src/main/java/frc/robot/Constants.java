// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Auto
    {
        public static final double K_XController = 1;
        public static final double K_YController = 1;
        public static final double K_RController = 1;
    }

    public static final class DrivetrainConstants{
        /**
         * The left-to-right distance between the drivetrain wheels
         * Should be measured from center to center.
         */
        //25 inches converted to 0.635 meters
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635; //Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        //25 inches converted to 0.635 meters
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635; //Measure and set wheelbase

        //FIXME CANCoder Ids and steer offset
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; 
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; 
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 16; 
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(261.1); //Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8; 
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; 
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(239.3); //Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; 
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 15; 
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(170.1); //Measure and set back left steer offset
        
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6; 
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; 
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14; 
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(87.8); //Measure and set back right steer offset

        //FeedForwardGains        
        public static final double ky = 0;
        public static final double kv = 0;
        public static final double ka = 0;
    }

    public static final class CameraConstants{
        public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("photonvision");
    
        public static final NetworkTableEntry tx = table.getEntry("tx");
        public static final NetworkTableEntry ty = table.getEntry("ty");
        public static final NetworkTableEntry ta = table.getEntry("ta");
        public static final NetworkTableEntry tv = table.getEntry("tv");
        public static final NetworkTableEntry camMode = table.getEntry("driverMode");
        public static final NetworkTableEntry pipeline = table.getEntry("pipelineIndex");
        public static final NetworkTableEntry stream = table.getEntry("stream");
   
       public static final double CAMERA_HEIGHT_METERS = .266; //FIXME camera height in meters
       public static final double TEST_TARGET_HEIGHT_METERS = .381; //FIXME TEST APRIL TAG HEIGHT
       public static final double LOWEST_COMP_TARGET_HEIGHT_METERS = .36; // lowest april tag height in comp
       public static final double HIGHEST_COMP_TARGET_HEIGHT_METERS = .59; // highest april tag height in comp
       public static final double CAMERA_PITCH_RADIANS = 0.0; //FIXME Camera tilt in radians
    }
}

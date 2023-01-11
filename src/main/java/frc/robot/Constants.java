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
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    //16.25 inches converted to 0.41275 meters
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.41275; //Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    //24.25 inches converted to 0.61595 meters
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.61595; //Measure and set wheelbase

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

     //LIMELIGHT
     public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    
     public static final NetworkTableEntry tx = table.getEntry("tx");
     public static final NetworkTableEntry ty = table.getEntry("ty");
     public static final NetworkTableEntry ta = table.getEntry("ta");
     public static final NetworkTableEntry tv = table.getEntry("tv");
     public static final NetworkTableEntry ledMode = table.getEntry("ledMode");
     public static final NetworkTableEntry camMode = table.getEntry("camMode");
     public static final NetworkTableEntry pipeline = table.getEntry("pipeline");
     public static final NetworkTableEntry stream = table.getEntry("stream");
}

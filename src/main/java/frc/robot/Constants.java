// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        public static final XboxController DRIVE_GAMEPAD = new XboxController(0);
//        public static final XboxController SECONDARY_GAMEPAD = new XboxController(1);
    }
    public static final class DrivetrainConstants {
        // Mechanical Constants
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;
        public static final double WHEEL_DIAMETER = 0.0968375;
        public static final double GEAR_RATIO = 12.8;

        // PORT #s and OFFSETS
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(268.77);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.54);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 16;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.22);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 15;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(79.81);

        public final static double MAX_VOLTAGE = 12;// 12

        // Theoritcal speed cap XY
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 11000.0 / 60.0 *
                SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
                SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

        // Theorital rot speed cap
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        // Max speed is scary
        public static final double SPEED_LIMIT = .3;

        public static final class Auto
        {
            public static final PIDConstants XY_CONSTANTS = new PIDConstants(.4, 0, 0); // FIXME PID CONSTANTS
            public static final PIDConstants THEATA_CONSTANTS = new PIDConstants(0, 0, 0); // FIXME PID constants THETA
            public static final PIDController XY_CONTTROLLER = new PIDController(.4,0,0);
            public static final PIDController THEATA_CONTTROLLER = new PIDController(0,0,0);
        }
    }

    public static final class CameraConstants {
        public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision")
                .getSubTable("photonvision");

        public static final NetworkTableEntry targetPixelsX = table.getEntry("targetPixelsX");
        public static final NetworkTableEntry targetPixelsY = table.getEntry("targetPixelsY");
        public static final NetworkTableEntry targetYaw = table.getEntry("targetYaw");

        public static final NetworkTableEntry camMode = table.getEntry("driverMode");
        public static final NetworkTableEntry pipeline = table.getEntry("pipelineIndex");
        public static final NetworkTableEntry stream = table.getEntry("stream");

        // FIXME camera height in meters on the bot
        public static final double CAMERA_HEIGHT_METERS = .266;
        public static final double LOWEST_TARGET_HEIGHT = .36;
        public static final double HIGHEST_TARGET_HEIGHT = .59;

        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0,0,0),new Rotation3d());

        // FIXME camera angle in degrees on the bot
        public static final double CAMERA_ANGLE = 0;

        public static final double CAMERA_PITCH_RADIANS = Math.toRadians(CAMERA_ANGLE);
    }

    //FIXME subsystem new CAN IDs
    public static final class ArmConstants{
        public static final int EXTENDING_MOTOR = 0;
        public static final int RAISING_MOTOR = 0;
        public static final int STRING_PLOT_CHANELLE = 0;
        public static final PIDController EXTENDER_CONTROLLER = new PIDController(0,0,0); // FIXME needs to be tuned
        public static final PIDController PIVOT_CONTROLLER = new PIDController(0,0,0); // FIXME needs to be tuned
        /*
            you need to measure the string plot value for the extender to score for each of the possible positions
            end you need to set the default value of the extender and the arm
         */
        public static final double EXTENDER_LENGTHS[] = {0,0,0,0}; // FIXME needs to be actual measurements look above for instruction
        /*
            for the pivot height you need to measure the value of the continues encoder as the arm gets to the height needed to place a cone at every position
         */
        public static final double PIVOT_HEIGHT[] = {0,0,0,0}; // FIXME needs to be actual measurements look above for instruction

    }
    public static final class ConveyorConstants{
        public static final int CONVEYOR_MOTOR = 0;
    }
    public static final class GripperConstants{
        public static final int GRIPPER_MOTOR = 1;

        public static final PIDConstants GRIPPER_PID_CONSTANTS = new PIDConstants(.1, 0, 0);
    }
    public static final class IntakeConstants{

        public static final int LEFT_INTAKE_MOTOR = 0;
        public static final int RIGHT_INTAKE_MOTOR = 0;

        public static final int RUN_INTAKE = 1;
        public static final int STOP_INTAKE = 0;
        public static final int RUN_INVERTED_INTAKE = 2;

        public static final double INTAKE_SPEED = .5;

    }
}

package frc.robot;

import com.controller.StadiaController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PneumaticHub;

public final class Constants {
	public final static PneumaticHub PNEUMATIC_HUB = new PneumaticHub(18);

	public static final class IOConstants {
		public static final StadiaController DRIVE_GAMEPAD = new StadiaController(0);
		public static final StadiaController SECONDARY_GAMEPAD = new StadiaController(1);
	}

	public static final class DrivetrainConstants {

		public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635;
		public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;
		public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

		// PORT #s and OFFSETS
		public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6;
		public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
		public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 15;
		public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.30);

		public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
		public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
		public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
		public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.62);

		public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
		public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1;
		public static final int BACK_LEFT_MODULE_STEER_ENCODER = 16;
		public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.22);

		public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
		public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
		public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14;
		public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(79.63);

		public static final double NORMAL_SPEED = 12; // Units volts
		public static final double SLOW_SPEED = 6;
	}

	public static final class ArmConstants {

		public static final int EXTENDING_MOTOR = 11;
		public static final int PIVOT_MOTOR = 12;
		public static final int STRING_POT_CHANNEL = 0;
	}

	public static final class ConveyorConstants {
		public static final int CONVEYOR_MOTOR = 10;
		public static final double MOTOR_SPEED = .75;
	}

	public static final class GripperConstants {
		public static final int GRIPPER_MOTOR = 14;
	}

	public static final class IntakeConstants {
		public static final int LEFT_INTAKE_MOTOR = 9;
		public static final int RIGHT_INTAKE_MOTOR = 15;

		public static final double INTAKE_MOTOR_SPEED = .75;
	}
}

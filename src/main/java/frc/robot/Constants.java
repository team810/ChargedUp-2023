package frc.robot;

import com.controller.StadiaController;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {

	public final static PneumaticHub PNEUMATIC_HUB = new PneumaticHub(18);

	public static final class OIConstants {
		public static final XboxController DRIVE_GAMEPAD = new XboxController(0);
		// public static final Joystick SECONDARY_GAMEPAD = new Joystick(1);

		// public static final StadiaController DRIVE_GAMEPAD = new StadiaController(0);
		public static final StadiaController SECONDARY_GAMEPAD = new StadiaController(1);
	}

	public static final class DrivetrainConstants {

		// Mechanical Constants
		// 0.5969 roughly
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
		public static final double WHEEL_DIAMETER = 0.0968375;
		public static final double GEAR_RATIO = 8.16;

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
		public static double SPEED_LIMIT = .8;

		public static final class Auto {
			// TeleOP
			public static final PIDController XY_CONTROLLER = new PIDController(.2, .01, .01);
			public static final PIDController THETA_CONTROLLER = new PIDController(.1, 0, 0);

			// Auto
			public static final PIDConstants XY_CONSTANTS = new PIDConstants(25, 0, 0);
			public static final PIDConstants THETA_CONSTANTS = new PIDConstants(15, 0, 0);
			public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(.8, 3);
		}
	}

	public static final class CameraConstants {
		// Network entries
		public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision")
				.getSubTable("photonvision");

		public static final NetworkTableEntry targetPixelsX = table.getEntry("targetPixelsX");
		public static final NetworkTableEntry targetPixelsY = table.getEntry("targetPixelsY");
		public static final NetworkTableEntry targetYaw = table.getEntry("targetYaw");

		public static final NetworkTableEntry camMode = table.getEntry("driverMode");
		public static final NetworkTableEntry pipeline = table.getEntry("pipelineIndex");
		public static final NetworkTableEntry stream = table.getEntry("stream");

		// Shuffleboard
		public static final ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

		public static final ShuffleboardLayout CAMERA_VALUES = tab
				.getLayout("Limelight Values", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(0, 0);

		public static final double CAMERA_HEIGHT_METERS = .155;

		public static final double LOWEST_TARGET_HEIGHT = .36;
		public static final double HIGHEST_TARGET_HEIGHT = 1.06;

		public static final double CAMERA_ANGLE = 0;

		public static final double CAMERA_PITCH_RADIANS = Math.toRadians(CAMERA_ANGLE);
	}

	public static final class ArmConstants {

		// EXTENDER
		public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");

		public static final ShuffleboardLayout EXTENDER = ARM_TAB.getLayout("EXTENDER", BuiltInLayouts.kList)
				.withPosition(0, 0).withSize(2,
						4);
		public static final int EXTENDING_MOTOR = 11;
		public static final PIDController EXTENDER_CONTROLLER = new PIDController(0.15, 0, 0);

		// PIVOT
		public static final ShuffleboardLayout PIVOT = ARM_TAB.getLayout("PIVOT", BuiltInLayouts.kList)
				.withPosition(2, 0).withSize(2, 4);

		public static final int PIVOT_MOTOR = 12;
		public static final PIDController PIVOT_CONTROLLER = new PIDController(0.1, 0, 0);

		// String Pot
		public static final int STRING_POT_CHANNEL = 0;
	}

	public static final class ConveyorConstants {
		// Shuffleboard
		public static final ShuffleboardTab conveyorTab = Shuffleboard.getTab("Conveyor");
		public static final ShuffleboardLayout CONVEYOR_LAYOUT = conveyorTab
				.getLayout("Motor Values", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);

		// Motors
		public static final int CONVEYOR_MOTOR = 10;

		public static final double MOTOR_SPEED = .40;
	}

	public static final class GripperConstants {
		// Shuffleboard
		public static final ShuffleboardTab gripperTab = Shuffleboard.getTab("Gripper");
		public static final ShuffleboardLayout GRIPPER_M_VALUES = gripperTab
				.getLayout("Motor Values", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
		public static final ShuffleboardLayout GRIPPER_PID = gripperTab
				.getLayout("PID Values", BuiltInLayouts.kList)
				.withPosition(2, 0).withSize(2, 4);

		// Ports
		public static final int GRIPPER_MOTOR = 14;

		public static final int GRIPPER_SOL = 8;

		public static final int LIMIT_SWITCH = 2;

		public static final PIDController GRIPPER_CONTROLLER = new PIDController(.15, 0, 0);
	}

	public static final class IntakeConstants {
		// Shuffleboard
		public static final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
		public static final ShuffleboardLayout INTAKE_VALUES = intakeTab
				.getLayout("Motor Values", BuiltInLayouts.kList)
				.withPosition(0, 0).withSize(2, 4);

		public static final int LEFT_INTAKE_MOTOR = 9;
		public static final int RIGHT_INTAKE_MOTOR = 15;

		public static final double INTAKE_MOTOR_SPEED = .45;
	}

	public static final class ColorSensorConstants {
		// Shuffleboard
		public static final ShuffleboardTab colorsensorTab = Shuffleboard.getTab("Color Sensor");

		public static final ShuffleboardLayout COLOR_SENSOR = colorsensorTab
				.getLayout("Color Sensor", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(0, 0);
	}
}

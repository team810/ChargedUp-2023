package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

	private Speed speed_mode;
	// 4 modules on the drivetrain
	private final SwerveModule frontLeftModule;
	private final SwerveModule frontRightModule;
	private final SwerveModule backLeftModule;
	private final SwerveModule backRightModule;

	// Gyroscope
	private final AHRS m_navx = new AHRS(Port.kMXP);
	// Used to keep a virtual position of the robot
	private final SwerveDriveOdometry odometry;
	// Contains the current distances and angles of each module
	private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
	// Contains our speeds
	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	private SwerveModuleState[] moduleStates = {
			new SwerveModuleState(),
			new SwerveModuleState(),
			new SwerveModuleState(),
			new SwerveModuleState()
	};

	public Drivetrain() {
		speed_mode = Speed.Normal;

		m_navx.reset();
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		frontLeftModule = Mk3SwerveModuleHelper.createNeo(
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
				DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
				DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
				DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

		frontRightModule = Mk3SwerveModuleHelper.createNeo(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
				DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
				DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
		backLeftModule = Mk3SwerveModuleHelper.createNeo(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
				DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
				DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
				DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
		backRightModule = Mk3SwerveModuleHelper.createNeo(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(6, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
				DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
				DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
				DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET
		);

		zeroGyroscope();

		modulePositions[0] = new SwerveModulePosition();
		modulePositions[1] = new SwerveModulePosition();
		modulePositions[2] = new SwerveModulePosition();
		modulePositions[3] = new SwerveModulePosition();

		odometry = new SwerveDriveOdometry(DrivetrainConstants.KINEMATICS,
				new Rotation2d(0), modulePositions);

		resetPose(new Pose2d(0, 0, new Rotation2d(0)));

	}

	// Resetting
	public void zeroGyroscope() {
		m_navx.zeroYaw();

	}

	public void resetPose(Pose2d pose) {
		modulePositions = new SwerveModulePosition[]{
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition()
		};

		frontLeftModule.getDriveEncoder().setPosition(0);
		frontRightModule.getDriveEncoder().setPosition(0);
		backLeftModule.getDriveEncoder().setPosition(0);
		backRightModule.getDriveEncoder().setPosition(0);

		odometry.resetPosition(getGyroscopeRotation(), modulePositions, pose);
	}

	// We set the speeds we want from joystick values
	private void setSpeeds(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] states = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.NORMAL_SPEED);
		// Update virtual states
		moduleStates = states;
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	public void setStates(SwerveModuleState[] state) {

		if (speed_mode == Speed.Normal)
		{
			frontLeftModule.set(
					(- state[0].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[0].angle.getRadians());
			frontRightModule.set(
					( -state[1].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[1].angle.getRadians());
			backLeftModule.set(
					( - state[2].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[2].angle.getRadians());
			backRightModule.set(
					(-state[3].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[3].angle.getRadians());
		}
		if (speed_mode == Speed.Slow)
		{
			frontLeftModule.set(
					(- state[0].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[0].angle.getRadians());
			frontRightModule.set(
					( -state[1].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[1].angle.getRadians());
			backLeftModule.set(
					( - state[2].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[2].angle.getRadians());
			backRightModule.set(
					(-state[3].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[3].angle.getRadians());
		}


		SmartDashboard.putNumber("Front Left Speed", state[0].speedMetersPerSecond);
		SmartDashboard.putNumber("Front Right Speed", state[1].speedMetersPerSecond);
		SmartDashboard.putNumber("Back Left Speed", state[2].speedMetersPerSecond);
		SmartDashboard.putNumber("Back Right Speed", state[3].speedMetersPerSecond);
		SmartDashboard.putString("Mode", getSpeed_mode().toString());
	}

	public void setStatesAuto(SwerveModuleState[] state)
	{
		if (speed_mode == Speed.Normal)
		{
			frontLeftModule.set(
					(- state[0].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[0].angle.getRadians());
			frontRightModule.set(
					( -state[1].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[1].angle.getRadians());
			backLeftModule.set(
					( - state[2].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[2].angle.getRadians());
			backRightModule.set(
					(-state[3].speedMetersPerSecond
							* DrivetrainConstants.NORMAL_SPEED),
					state[3].angle.getRadians());
		}
		if (speed_mode == Speed.Slow)
		{
			frontLeftModule.set(
					(- state[0].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[0].angle.getRadians());
			frontRightModule.set(
					( -state[1].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[1].angle.getRadians());
			backLeftModule.set(
					( - state[2].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[2].angle.getRadians());
			backRightModule.set(
					(-state[3].speedMetersPerSecond
							* DrivetrainConstants.SLOW_SPEED),
					state[3].angle.getRadians());
		}

		SmartDashboard.putNumber("Front Left Speed", state[0].speedMetersPerSecond);
		SmartDashboard.putNumber("Front Right Speed", state[1].speedMetersPerSecond);
		SmartDashboard.putNumber("Back Left Speed", state[2].speedMetersPerSecond);
		SmartDashboard.putNumber("Back Right Speed", state[3].speedMetersPerSecond);
	}

	public double getPitch() {
		return m_navx.getPitch();
	}

	public SwerveModulePosition getPosition(int moduleNumber) {

//		return new SwerveModulePosition(
//				modules[moduleNumber].getDriveEncoder().getPosition(),
//				new Rotation2d(modules[moduleNumber].getSteerAngle()));
		return new SwerveModulePosition(); // FIXME need to actually write good code !!!
	}

	public void fast() {
		setSpeed_mode(Speed.Normal);
	}


	public void slow() {
		setSpeed_mode(Speed.Slow);
	}

	@Override
	public void periodic() {
		SmartDashboard.putData("Navx", m_navx);
		if (RobotState.isTeleop()) {
			setSpeeds(this.m_chassisSpeeds);
			setStates(moduleStates);
		}

		// updating odometry
		modulePositions[0] = getPosition(0);
		modulePositions[1] = getPosition(1);
		modulePositions[2] = getPosition(2);
		modulePositions[3] = getPosition(3);

		odometry.update(getGyroscopeRotation(), modulePositions);
	}

	public Rotation2d getGyroscopeRotation() {
		return m_navx.getRotation2d();
	}
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}
	public Speed getSpeed_mode() {
		return speed_mode;
	}

	public void setSpeed_mode(Speed mSpeed_mode) {
		speed_mode = mSpeed_mode;
	}
}

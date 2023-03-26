package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ChargeStationCommand extends CommandBase {
	private final Drivetrain drivetrain;
	private final PIDController controller = new PIDController(-.25, 0, 0); // FIXME tune PID controller
	private final double tolerance = 10;

	public ChargeStationCommand(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		controller.enableContinuousInput(-180, 180); // get pitch returns a value from -180 to 180
		controller.setTolerance(tolerance);

		controller.setSetpoint(0);
		drivetrain.lockWheels();
		addRequirements(this.drivetrain);
	}

	@Override
	public void initialize() {
		drivetrain.lockWheels();
	}

	@Override
	public void execute() {

		double yInput;

		yInput = controller.calculate(drivetrain.getPitch(), 0);

		yInput = Math.min(20, Math.max(-20, yInput));

		ChassisSpeeds speeds = new ChassisSpeeds(yInput, 0, 0);
		if (RobotState.isAutonomous()) {
			SwerveModuleState[] states = Constants.DrivetrainConstants.KINEMATICS.toSwerveModuleStates(speeds);
			SwerveDriveKinematics.desaturateWheelSpeeds(states,
					Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

			drivetrain.setStates(states);
		}
		if (RobotState.isTeleop()) {
			drivetrain.drive(speeds);
		}
	}

	@Override
	public void end(boolean interrupted) {

		drivetrain.drive(new ChassisSpeeds(0, 0, 0));
		drivetrain.unlockWheels();
	}

	private void drive(ChassisSpeeds speeds) {
		if (RobotState.isAutonomous()) {
			SwerveModuleState[] states = Constants.DrivetrainConstants.KINEMATICS.toSwerveModuleStates(speeds);
			SwerveDriveKinematics.desaturateWheelSpeeds(states,
					Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

			drivetrain.setStates(states);
		}
		if (RobotState.isTeleop()) {
			drivetrain.drive(speeds);
		}
	}

	@Override
	public boolean isFinished() {

		return controller.atSetpoint();
	}
}

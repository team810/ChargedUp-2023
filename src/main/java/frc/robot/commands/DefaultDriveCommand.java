package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Speed;
import lib.Deadband;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
	private final Drivetrain m_drivetrainSubsystem;

	private final DoubleSupplier xSupplier;
	private final DoubleSupplier ySupplier;
	private final DoubleSupplier zSupplier;

	private final Deadband xDeadband;
	private final Deadband yDeadband;
	private final Deadband zDeadband;

	public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
	                           DoubleSupplier translationXSupplier,
	                           DoubleSupplier translationYSupplier,
	                           DoubleSupplier rotationSupplier) {
		this.m_drivetrainSubsystem = drivetrainSubsystem;
		this.xSupplier = translationXSupplier;
		this.ySupplier = translationYSupplier;
		this.zSupplier = rotationSupplier;

		xDeadband = new Deadband(.02, 0);
		yDeadband = new Deadband(.02, 0);
		zDeadband = new Deadband(.02, 0);

		addRequirements(drivetrainSubsystem);
	}

	@Override
	public void execute() {

		double x;
		double y;
		double z;

		// Moving the input into a local var
		x = xSupplier.getAsDouble();
		y = ySupplier.getAsDouble();
		z = zSupplier.getAsDouble();

		// Deadband needs to be applied to raw input
		x = xDeadband.apply(x);
		y = yDeadband.apply(y);
		z = zDeadband.apply(z);

		// This is adding a curve
		x = Math.pow(x, 3);
		y = Math.pow(y, 3);
		z = Math.pow(z, 3);

		// Multiplying to make the units into meters per second
		if (m_drivetrainSubsystem.getSpeed_mode() == Speed.Normal)
		{
			x = x * Constants.DrivetrainConstants.NORMAL_SPEED;
			y = y * Constants.DrivetrainConstants.NORMAL_SPEED;
			z = z * Constants.DrivetrainConstants.NORMAL_SPEED;
		}
		if (m_drivetrainSubsystem.getSpeed_mode() == Speed.Slow)
		{
			x = x * Constants.DrivetrainConstants.SLOW_SPEED;
			y = y * Constants.DrivetrainConstants.SLOW_SPEED;
			z = z * Constants.DrivetrainConstants.SLOW_SPEED;
		}

		if (RobotState.isTeleop())
		{
			m_drivetrainSubsystem.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							x,
							y,
							z,
							m_drivetrainSubsystem.getGyroscopeRotation()));
		}

	}

	@Override
	public void end(boolean interrupted) {
		m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}

}

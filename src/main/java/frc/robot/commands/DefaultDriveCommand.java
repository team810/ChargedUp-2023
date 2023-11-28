package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.Drivetrain;
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

		xDeadband = new Deadband(.02);
		yDeadband = new Deadband(.02);
		zDeadband = new Deadband(.1);

		addRequirements(drivetrainSubsystem);
	}

	@Override
	public void execute() {

		double x;
		double y;
		double z;

		// Moving the input into a local var
		y = -xSupplier.getAsDouble();
		x = -ySupplier.getAsDouble();
		z = -zSupplier.getAsDouble();

		// Deadband needs to be applied to raw input
		x = xDeadband.apply(x);
		y = yDeadband.apply(y);
		z = zDeadband.apply(z);

		// This is adding a curve
		x = Math.pow(x, 3);
		y = Math.pow(y, 3);
		z = Math.pow(z, 3);


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

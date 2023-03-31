package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
	private final Intake m_intake = new Intake();
	private final Conveyor m_conveyor = new Conveyor();
	private final Gripper m_gripper = new Gripper();
	private final Limelight m_lime = new Limelight();
	private final HardStopSubsystem m_hardStop = new HardStopSubsystem();
	private final Drivetrain m_drive = new Drivetrain();
	private final Arm m_arm = new Arm();
	// private final Autos m_autos = new Autos(m_drive, m_intake, m_conveyor, m_arm, m_gripper, m_lime, m_hardStop);
	private final Autos m_autos = new Autos(m_drive, m_intake, m_conveyor, m_arm, m_gripper, m_hardStop);

	public RobotContainer() {
		CameraServer.startAutomaticCapture();
		// m_lime.setMode("Reflective Tape");

		m_drive.setDefaultCommand(new DefaultDriveCommand(
				m_drive,
				() -> -modifyAxis(OIConstants.DRIVE_GAMEPAD.getLeftY() *
						DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConstants.SPEED_LIMIT),
				() -> -modifyAxis(OIConstants.DRIVE_GAMEPAD.getLeftX() *
						DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConstants.SPEED_LIMIT),
				() -> -modifyAxis(
						OIConstants.DRIVE_GAMEPAD.getRightX() *
								DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * .65));

		m_gripper.setDefaultCommand(
				new GripperSetpoint(m_gripper, () -> OIConstants.SECONDARY_GAMEPAD.getRawAxis(1) * .45));
		m_conveyor.setDefaultCommand(
				new ConveyorCommand(m_conveyor, () -> OIConstants.SECONDARY_GAMEPAD.getRawAxis(4)));

		m_arm.setDefaultCommand(
				new ArmCommand(
						m_arm,
						OIConstants.SECONDARY_GAMEPAD::getPOV
				)
		);
		configureButtonBindings();
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, .2);
		// Cubed the axis, smoother driving
		value = Math.pow(value, 3);

		return value;
	}

	void primaryButtons()
	{
		new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(6)).onTrue(
				new InstantCommand(m_drive::zeroGyroscope));

		// switch between slow and fast mode
		new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(5)).whileTrue(
				new InstantCommand(
						() -> m_drive.slow()
				)
		);
		new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getLeftTriggerAxis() > .75).onTrue(
				new InstantCommand(
						() -> m_drive.fast()
				)
		);
	}
	private void configureButtonBindings() {
		primaryButtons();
		
		//Zero gyro

		// Primary
		// Intake forward

		new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRightTriggerAxis() > .75).whileTrue(
				new StartEndCommand(
						m_intake::runIntake,
						m_intake::stopIntake,
						m_intake).alongWith(
						new StartEndCommand(
								() -> {
									m_conveyor.setReversed(false);
									m_conveyor.setEnabled(true);
								},
								() -> {
									m_conveyor.setReversed(false);
									m_conveyor.setEnabled(false);
								},
								m_conveyor)));

		//Secondary

		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(6)).whileTrue(
				new StartEndCommand(
						() -> m_hardStop.in(),
						() -> m_hardStop.out(),
						m_hardStop
				)
		);

		// Intake Reverse
		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getYButton()).whileTrue(
				new ParallelCommandGroup(
						new StartEndCommand(
								m_intake::runIntakeReversed,
								m_intake::stopIntake,
								m_intake),
						new StartEndCommand(
								() -> {
									m_conveyor.setReversed(true);
									m_conveyor.setEnabled(true);
								},
								() -> {
									m_conveyor.setReversed(false);
									m_conveyor.setEnabled(false);
								},
								m_conveyor)));

		// Intake forward
		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getAButton()).whileTrue(
				new StartEndCommand(
						m_intake::runIntake,
						m_intake::stopIntake,
						m_intake).alongWith(
						new StartEndCommand(
								() -> {
									m_conveyor.setReversed(false);
									m_conveyor.setEnabled(true);
								},
								() -> {
									m_conveyor.setReversed(false);
									m_conveyor.setEnabled(false);
								},
								m_conveyor)));

		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getBButton()).onTrue(
				new InstantCommand(() -> m_hardStop.in()));

		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getXButton()).toggleOnTrue(
				new InstantCommand(() -> m_hardStop.out()));
		// Medium score
		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(5))
				.toggleOnTrue(
						new SequentialCommandGroup(
								new InstantCommand(m_hardStop::out),
//								new ToTargetCommand(m_conveyor,m_drive,m_lime),
								new RaiseArmCommand(m_arm, 2, 1)
						)
				);
		// High score
		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(12))
				.toggleOnTrue(
						new SequentialCommandGroup(
								new InstantCommand(m_hardStop::out),
//							new ToTargetCommand(m_conveyor,m_drive,m_lime),
								new RaiseArmCommand(m_arm, 3, 1)
						)
				);

		// Run after scoring
		new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(13)).toggleOnTrue(
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								new InstantCommand(() -> {
									m_arm.restExtender();
									m_hardStop.out();
								}),
								new WaitCommand(.75),
								new InstantCommand(() -> m_arm.restPivot()))));


//		new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRightTriggerAxis() > .75).onTrue(
//				new ToTargetCommand(m_conveyor, m_drive, m_lime)
//		);
		new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getAButton()).onTrue(
				new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
		);

	}

	public void teleopInit() {
		m_arm.restExtender();
		m_arm.restPivot();

//		CommandScheduler.getInstance().schedule(
//				new SequentialCommandGroup(
//						new WaitCommand(1),
//
//				)
//		);
	}

	public Command getAutonomousCommand() {

//		return m_autos.genPath("RedRight2");
		return null;
		// return new ScoreCommand(m_arm, m_drive, m_gripper, m_conveyor, m_intake, 3, 2);
		// return null;
	}
}
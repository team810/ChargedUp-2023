package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Speed;

public class RobotContainer {
	private final Intake m_intake = new Intake();
	private final Conveyor m_conveyor = new Conveyor();
	private final Gripper m_gripper = new Gripper();

	private final HardStopSubsystem m_hardStop = new HardStopSubsystem();
	private final Drivetrain m_drive = new Drivetrain();
	private final Arm m_arm = new Arm();

	public RobotContainer() {

		CameraServer.startAutomaticCapture();
		CameraServer.startAutomaticCapture();


		m_drive.setDefaultCommand(new DefaultDriveCommand(
				m_drive,
				IOConstants.DRIVE_GAMEPAD::getLeftX,
				IOConstants.DRIVE_GAMEPAD::getLeftY,
				IOConstants.DRIVE_GAMEPAD::getRightX
				));

		m_gripper.setDefaultCommand(
				new GripperSetpoint(m_gripper, () -> IOConstants.SECONDARY_GAMEPAD.getRawAxis(1) * .45));
		m_conveyor.setDefaultCommand(
				new ConveyorCommand(m_conveyor, () -> IOConstants.SECONDARY_GAMEPAD.getRawAxis(4)));

		m_arm.setDefaultCommand(
				new ArmCommand(
						m_arm,
						IOConstants.SECONDARY_GAMEPAD::getPOV
				)
		);

		configureButtonBindings();
	}
	void primaryButtons()
	{
		new Trigger(() -> IOConstants.DRIVE_GAMEPAD.getRawButton(6)).onTrue(
				new InstantCommand(m_drive::zeroGyroscope));

		new Trigger(() -> IOConstants.DRIVE_GAMEPAD.getRawButton(5)).onTrue(
				new InstantCommand(
						() -> m_drive.setSpeed_mode(Speed.Normal)
				)
		);
		new Trigger(() -> IOConstants.DRIVE_GAMEPAD.getRawButton(13)).onTrue(
				new InstantCommand(
						() -> m_drive.setSpeed_mode(Speed.Slow)
				)
		);
		new Trigger(IOConstants.DRIVE_GAMEPAD::getAButton).onTrue(
				new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
		);
	}

	void secondaryConfig()
	{
//		new Trigger(() -> IOConstants.DRIVE_GAMEPAD.getRightTriggerAxis() > .75).whileTrue(
//				new StartEndCommand(
//						m_intake::runIntake,
//						m_intake::stopIntake,
//						m_intake).alongWith(
//						new StartEndCommand(
//								() -> {
//									m_conveyor.setReversed(false);
//									m_conveyor.setEnabled(true);
//								},
//								() -> {
//									m_conveyor.setReversed(false);
//									m_conveyor.setEnabled(false);
//								},
//								m_conveyor)));

		new Trigger(() -> IOConstants.SECONDARY_GAMEPAD.getRawButton(6)).whileTrue(
				new StartEndCommand(
						m_hardStop::in,
						m_hardStop::out,
						m_hardStop
				)
		);

		// Intake Reverse
		new Trigger(IOConstants.SECONDARY_GAMEPAD::getYButton).whileTrue(
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
		new Trigger(IOConstants.SECONDARY_GAMEPAD::getAButton).whileTrue(
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

		new Trigger(IOConstants.SECONDARY_GAMEPAD::getBButton).onTrue(
				new InstantCommand(m_hardStop::in));

		new Trigger(IOConstants.SECONDARY_GAMEPAD::getXButton).toggleOnTrue(
				new InstantCommand(m_hardStop::out));
		// Medium score
		new Trigger(() -> IOConstants.SECONDARY_GAMEPAD.getRawButton(5))
				.toggleOnTrue(
						new SequentialCommandGroup(
								new InstantCommand(m_hardStop::out),
								new RaiseArmCommand(m_arm, 2, 1)
						)
				);
		// High score
		new Trigger(() -> IOConstants.SECONDARY_GAMEPAD.getRawButton(12))
				.toggleOnTrue(
						new SequentialCommandGroup(
								new InstantCommand(m_hardStop::out),
								new RaiseArmCommand(m_arm, 3, 1)
						)
				);
		// Run after scoring
		new Trigger(() -> IOConstants.SECONDARY_GAMEPAD.getRawButton(13)).toggleOnTrue(
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								new InstantCommand(() -> {
									m_arm.restExtender();
									m_hardStop.out();
								}),
								new WaitUntilCommand(m_arm::extenderAtSetpoint),
								new InstantCommand(m_arm::restPivot))));

	}
	private void configureButtonBindings() {
		primaryButtons();
		secondaryConfig();
	}

	public void teleopInit() {
		m_arm.restExtender();
		m_arm.restPivot();
	}

	public Command getAutonomousCommand(){
		return new ScoreCommand(m_arm, m_drive, m_gripper, m_conveyor, m_intake, 3,2,m_hardStop);
	}
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

import java.util.function.DoubleSupplier;

public class GripperSetpoint extends CommandBase {
	private final DoubleSupplier speed;
	Gripper m_gripper;

	/**
	 * Creates a new GripperSetpoint.
	 */
	public GripperSetpoint(Gripper m_gripper, DoubleSupplier speed) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.m_gripper = m_gripper;
		this.speed = speed;

		addRequirements(m_gripper);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (RobotState.isTeleop()) {
			m_gripper.setMotor(speed.getAsDouble());
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

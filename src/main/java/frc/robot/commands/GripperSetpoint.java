// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GripperSetpoint extends CommandBase {
  Gripper m_gripper;
  private DoubleSupplier speed;
  /** Creates a new GripperSetpoint. */
  public GripperSetpoint(Gripper m_gripper, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_gripper = m_gripper;
    this.speed = speed;

    addRequirements(m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotState.isTeleop())
    {
      m_gripper.setPoint(Math.min(Math.max((m_gripper.getSetpoint() + speed.getAsDouble()),-12) , 10));
      // m_gripper.setPoint(m_gripper.getSetpoint() + speed.getAsDouble());
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ManualPivot extends CommandBase {
  private Arm m_arm;
  private Intake m_intake;
  private DoubleSupplier speed;

  /** Creates a new ManualPivot. */
  public ManualPivot(Arm m_arm, Intake m_intake, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_intake = m_intake;
    this.speed = speed;

    addRequirements(m_arm, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_intake.in();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.in();
    m_arm.runPivot(-this.speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

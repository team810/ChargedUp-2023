// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase swerveAutoTest(DrivetrainSubsystem swerve) {

    return Commands.sequence(
      new AutoDriveCommand(swerve, 0, 1, 90), 
      new WaitCommand(1),
      new AutoDriveCommand(swerve, 0, 0, 0)
    );

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

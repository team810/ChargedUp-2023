// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTTT extends PIDCommand {
  /** Creates a new TurnToTarget. */
  public AprilTTT(Drivetrain m_drive, Limelight lime) {
    super(
        // PID Values
        new PIDController(.05, 0.001, 0.0035),
        // Measurement is the yaw that the limelight returns
        () -> lime.getBestTarget().getYaw(),
        // Setpoint is always 0, as the delta x must be 0 for a line up
        () -> 0,
        // Output command
        output -> {
          new DefaultDriveCommand(m_drive, () -> 0.0, () -> 0.0, () -> output);
        });

    addRequirements(m_drive);
    // Set tolerance to +/- .2, as that is "good enough"
    getController().setTolerance(.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

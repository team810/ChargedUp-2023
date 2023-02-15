package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Arm;

public class RobotContainer {
//   private final Drivetrain m_drivetrainSubsystem = new Drivetrain();
//   private final Gripper m_gripper = new Gripper();
//   private final Conveyor m_conveyor = new Conveyor();
//   private final Intake m_intake = new Intake();
  private final Arm arm = new Arm();

  public RobotContainer() {

    // Set up the default command for the drivetrain.
//    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
//            m_drivetrainSubsystem,
//            () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
//            () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
//            () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getRightX())* DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
//    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Driving Constants
//    new Trigger(OIConstants.DRIVE_GAMEPAD::getRightBumper).whileTrue(
//            new InstantCommand(m_drivetrainSubsystem::zeroGyroscope)
//    );
//
//    // Grip Cone
//    new Trigger(OIConstants.DRIVE_GAMEPAD::getAButton).whileTrue(
//            new StartEndCommand(m_gripper::gripCone, m_gripper::rest, m_gripper)
//    );
//    new Trigger(OIConstants.DRIVE_GAMEPAD::getBButton).whileTrue(
//            new StartEndCommand(m_gripper::gripCube, m_gripper::rest, m_gripper)
//    );
//
//    // Intake
//    new Trigger(OIConstants.SECONDARY_GAMEPAD::getLeftBumper).whileTrue(
//            new StartEndCommand(() -> m_intake.runIntake(.5), () -> m_intake.runIntake(0), m_intake)
//    );
//    new Trigger(OIConstants.SECONDARY_GAMEPAD::getRightBumper).toggleOnTrue(
//            new InstantCommand(m_intake::actuateIntake)
//    );



    new Trigger(Constants.OIConstants.DRIVE_GAMEPAD::getAButton).whileTrue(new InstantCommand(arm::lowGoal));
    new Trigger(Constants.OIConstants.DRIVE_GAMEPAD::getBButton).whileTrue(new InstantCommand(arm::rest));




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
    value = deadband(value, .1);
    // Cubed the axis, smoother driving
    value = Math.pow(value, 3);

    return value * DrivetrainConstants.SPEED_LIMIT;
  }

  public Command getAutonomousCommand() {
    // return m_drivetrainSubsystem.forward();
    return null;
  }
}

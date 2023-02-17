package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final Drivetrain m_drive = new Drivetrain();
  private final Arm m_arm = new Arm();
 private final Intake m_intake = new Intake();
  private final Gripper m_gripper = new Gripper();
  private final Conveyor m_conveyor = new Conveyor();
  // private final ColorSensor colorSensor = new ColorSensor();
  private final Limelight m_lime = new Limelight();

  private final Autos autos = new Autos(m_drive, m_intake, m_conveyor, m_arm, m_gripper);


  public RobotContainer() {

    // Set up the default command for the drivetrain.
    m_drive.setDefaultCommand(new DefaultDriveCommand(
        m_drive,
        () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getRawAxis(1) *
            DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND),
        () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getRawAxis(0) *
            DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND),
        () -> -modifyAxis(
            OIConstants.DRIVE_GAMEPAD.getRawAxis(4) *
                DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  //Raise/Lower Arm
    new Trigger(OIConstants.DRIVE_GAMEPAD::getYButton).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(m_arm::lowGoal),
        new WaitCommand(1),
        new InstantCommand(m_arm::middleGoal),
        new WaitCommand(1),
        new InstantCommand(m_arm::highGoal),
        new WaitCommand(1),
        new InstantCommand(m_arm::rest)
    ));

    //Run conveyor
//    new Trigger(OIConstants.DRIVE_GAMEPAD::getAButton).whileTrue(
//      new StartEndCommand(()->m_conveyor.runConveyor(-.5),()-> m_conveyor.runConveyor(0), m_conveyor));
//    new Trigger(OIConstants.DRIVE_GAMEPAD::getXButton).whileTrue(
//      new StartEndCommand(()->m_conveyor.runConveyor(.5),()-> m_conveyor.runConveyor(0), m_conveyor));

    // Grip Cone
    new Trigger(OIConstants.DRIVE_GAMEPAD::getAButton).whileTrue(
        new StartEndCommand(m_gripper::gripCone, m_gripper::rest, m_gripper));
    // Grip Cube
    new Trigger(OIConstants.DRIVE_GAMEPAD::getBButton).whileTrue(
        new StartEndCommand(m_gripper::gripCube, m_gripper::rest, m_gripper));

    //Actuate intake
   new Trigger(OIConstants.DRIVE_GAMEPAD::getXButton).toggleOnTrue(
     new StartEndCommand(m_intake::actuateIntake, m_intake::actuateIntake, m_intake));

    // Zero gyroscope
    new Trigger(OIConstants.DRIVE_GAMEPAD::getBButton).onTrue(new InstantCommand(m_drive::zeroGyroscope));

    // Switch to AprilTag Pipeline
    new Trigger(OIConstants.DRIVE_GAMEPAD::getAButton).onTrue(new InstantCommand(() -> m_lime.setMode("AprilTag")));

    // Switch to Reflective Tape
    new Trigger(OIConstants.DRIVE_GAMEPAD::getBButton).onTrue(new InstantCommand(() -> m_lime.setMode("Reflective Tape")));

    // toggle conveyor on and off
    new Trigger(OIConstants.SECONDARY_GAMEPAD::getAButton).whileTrue(
            new StartEndCommand(
                    () -> m_conveyor.setDisabled(!m_conveyor.isDisabled()),
                    () -> m_conveyor.setDisabled(m_conveyor.isDisabled()),
                    m_conveyor
            )
    );

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

    return value * DrivetrainConstants.SPEED_LIMIT;
  }

  public Command getAutonomousCommand() {
    // return m_drivetrainSubsystem.forward();
    return null;
  }
}

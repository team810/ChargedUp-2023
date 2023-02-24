package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final Drivetrain m_drive = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Gripper m_gripper = new Gripper();
  private final Conveyor m_conveyor = new Conveyor();
  private final Limelight m_lime = new Limelight();
//  private final Autos autos = new Autos(m_drive, m_intake, m_conveyor, m_arm, m_gripper, m_lime);

  public RobotContainer() {
    m_lime.setMode("Reflective Tape");
    // Set up the default command for the drivetrain.
    m_drive.setDefaultCommand(new DefaultDriveCommand(
        m_drive,
        () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getRawAxis(1) *
            DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.25),
        () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getRawAxis(0) *
            DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.25),
        () -> -modifyAxis(
            OIConstants.DRIVE_GAMEPAD.getRawAxis(3) *
                DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.25)));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(12)).onTrue(
            new InstantCommand(() -> m_drive.zeroGyroscope())
    );
    new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(13)).onTrue(
            new ParallelCommandGroup(
                    new StartEndCommand(
                            () -> m_intake.runIntake(-.45),
                            () -> m_intake.runIntake(0),
                            m_intake
                    )
            )
    );

    new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(2)).onTrue(
            new InstantCommand(m_intake::out)
    );

    new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(3)).onTrue(
            new InstantCommand(m_intake::in)
    );


    //Conveyor
    // new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(5)).whileTrue(
    //   new StartEndCommand(()-> m_conveyor.runConveyor(.2), ()-> m_conveyor.runConveyor(0), m_conveyor)
    // );
    // new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(6)).whileTrue(
    //   new StartEndCommand(()-> m_conveyor.runConveyor(-.2), ()-> m_conveyor.runConveyor(0), m_conveyor)
    // );
    // new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(5)).whileTrue(
    //   new StartEndCommand(()-> m_conveyor.runConveyorWithColor(), ()-> m_conveyor.runConveyor(0), m_conveyor)
    // );

    //Intake


    new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(4)).whileTrue(
                  new ParallelCommandGroup(
                    new StartEndCommand(
                      () -> m_intake.runIntake(.45),
                      () -> m_intake.runIntake(0),
                      m_intake
                    )
                  )
    );

    new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(5)).toggleOnTrue(
            new ScoreCommand(m_arm,m_drive, m_gripper,m_lime, m_conveyor, 2)
    );
    new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(6)).toggleOnTrue(
            new ScoreCommand(m_arm,m_drive, m_gripper,m_lime, m_conveyor, 3)
    );

//    //Gripper
//    new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(3)).onTrue(
//      new InstantCommand(m_gripper::rest)
//    );
//    new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(2)).onTrue(
//      new InstantCommand(m_gripper::gripCone)
//    );
//
//    //Extender
//    new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(5)).whileTrue(
//      new StartEndCommand(()-> m_arm.runExtender(.4), ()-> m_arm.runExtender(0), m_arm)
//    );
//    new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(6)).whileTrue(
//      new StartEndCommand(()-> m_arm.runExtender(-.4), ()-> m_arm.runExtender(0), m_arm)
//    );
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
//    return autos.genPath("NewPath");
//    return new ToTargetCommand(m_conveyor, m_drive, m_lime);
    return null;
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
   private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
   private final Limelight m_lime = new Limelight();

  // private final Joystick RIGHT = new Joystick(0);
  private final Joystick LEFT = new Joystick(1);  
  //private final PathPlannerTrajectory Path = PathPlanner.loadPath("New Path",4,3);
  private XboxController mController = new XboxController(0);

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // modifyAxis(0);
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //     m_drivetrainSubsystem,
    //     ()-> -modifyAxis(RIGHT.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     ()-> -modifyAxis(RIGHT.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     ()-> -modifyAxis(LEFT.getZ()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    

        //XboxControllera Controlls 
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        ()-> -modifyAxis(mController.getRawAxis(5)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        ()-> -modifyAxis(mController.getRawAxis(4)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        ()-> -modifyAxis(mController.getRawAxis(0)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
      
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Vison: 
    //AprilTag Pipeline
    // new JoystickButton(LEFT, 4).onTrue(new InstantCommand(()-> m_lime.setMode(0)));
    //Limelight Pipeline
    // new JoystickButton(LEFT, 2).onTrue(new InstantCommand(()-> m_lime.setMode(1)));
    //Processing
    // new JoystickButton(LEFT, 1).onTrue(new InstantCommand(()-> m_lime.setMode(2)));
    //Zeros the gyroscope
    // new JoystickButton(RIGHT, 1).onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));
    
    //Xboxcontroller Controlles 
    // Back button zeros the gyroscope on Xboxcontroller
    //new JoystickButton(mController, 5).onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));
    //Limelight buttons on Xboxcontroller
    //Apirltag
    new JoystickButton(mController, 1).onTrue(new InstantCommand(()-> m_lime.setMode(0)));
    //Limelight 
    new JoystickButton(mController, 2).onTrue(new InstantCommand(()-> m_lime.setMode(1)));
    //Processing 
    new JoystickButton(mController, 3).onTrue(new InstantCommand(()-> m_lime.setMode(2)));
    }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


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
    value = deadband(value, .5);

    // Square the axis
    value = Math.pow(value, 3);//Math.copySign(value * value, value);

    return value;
  }

  public Command getAutonomousCommand() {
    // PIDController Xcontrlor = new PIDController(Constants.Auto.K_XController,0,0);
    // PIDController YControlor = new PIDController(Constants.Auto.K_YController,0,0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(
    //         Constants.Auto.K_RController, 0,0, new TrapezoidProfile.Constraints(4,3)
    // );

    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Trajectory PathTrajectory = new Trajectory(Path.getStates());

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //         PathTrajectory,
    //         m_drivetrainSubsystem::GetPos,
    //         m_drivetrainSubsystem.get_kinematics(),
    //         Xcontrlor,
    //         YControlor,
    //         thetaController,
    //         m_drivetrainSubsystem::SetModuleStates,
    //         m_drivetrainSubsystem
    // );
    // return swerveControllerCommand;
    return null;

  }
}

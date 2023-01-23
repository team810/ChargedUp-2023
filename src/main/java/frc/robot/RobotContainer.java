// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

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
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final PathPlannerTrajectory Path = PathPlanner.loadPath("New Path",4,3);
  //private final Limelight m_lime = new Limelight();

  private final Joystick RIGHT = new Joystick(1);
  private final Joystick LEFT = new Joystick(0);

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    m_drivetrainSubsystem = new DrivetrainSubsystem(Path.getInitialPose()); // Pasing through init pos for odometry.

    modifyAxis(0);
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        ()-> -LEFT.getY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        ()-> -LEFT.getX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        ()-> -RIGHT.getZ() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
    );
      
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //Vison: 
    //AprilTag Long Pange Pipeline
//    new JoystickButton(LEFT, 4).onTrue(new InstantCommand(()-> m_lime.setMode(0)));
//    //Limelight Pipeline
//    new JoystickButton(LEFT, 2).onTrue(new InstantCommand(()-> m_lime.setMode(1)));
//    //AprilTag Short Range Pipeline
//    new JoystickButton(LEFT, 3).onTrue(new InstantCommand(()-> m_lime.setMode(2)));
//    //Processing
//    new JoystickButton(LEFT, 1).onTrue(new InstantCommand(()-> m_lime.setMode(3)));
//    // Back button zeros the gyroscope
    // new JoystickButton(RIGHT, 1).onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));
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
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public Command getAutonomousCommand() {

    // An example command will be run in autonomous
    PIDController Xcontrlor = new PIDController(Constants.Auto.K_XController,0,0);
    PIDController YControlor = new PIDController(Constants.Auto.K_YController,0,0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.Auto.K_RController, 0,0, new TrapezoidProfile.Constraints(4,3)
    );

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    Trajectory PathTrajectory = new Trajectory(Path.getStates());

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            PathTrajectory,
            m_drivetrainSubsystem::GetPos,
            m_drivetrainSubsystem.get_kinematics(),
            Xcontrlor,
            YControlor,
            thetaController,
            m_drivetrainSubsystem::SetModuleStates,
            m_drivetrainSubsystem
    );
    return swerveControllerCommand;

  }
}

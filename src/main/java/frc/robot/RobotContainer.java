package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

import java.util.ArrayList;
import java.util.HashMap;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
   private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
   private final Limelight m_lime = new Limelight();
    private final PathPlannerTrajectory Path = PathPlanner.loadPath("New Path",1,3);

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
        ()-> -modifyAxis(mController.getRawAxis(4)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        ()-> -modifyAxis(mController.getRawAxis(3)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
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
    value = deadband(value, .2);

    // Square the axis
    value = Math.pow(value, 3);//Math.copySign(value * value, value);

    return value;
  }

    public Command getAutonomousCommand() {
//        m_drivetrainSubsystem.zeroGyroscope();
//        m_drivetrainSubsystem.setStates(m_drivetrainSubsystem.get_kinematics().toSwerveModuleStates(new ChassisSpeeds(0,0,0)));
//
//        // An example command will be run in autonomous
//        PIDController Xcontrlor = new PIDController(Constants.Auto.K_XController,0,0);
//        PIDController YControlor = new PIDController(Constants.Auto.K_YController,0,0);
//        ProfiledPIDController thetaController = new ProfiledPIDController(
//                Constants.Auto.K_RController, 0,0, new TrapezoidProfile.Constraints(.1,.1)
//        );
//
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);
//        Trajectory PathTrajectory = new Trajectory(Path.getStates());
//
//
//        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//                PathTrajectory,
//                m_drivetrainSubsystem::getPos,
//                m_drivetrainSubsystem.get_kinematics(),
//                Xcontrlor,
//                YControlor,
//                thetaController,
//                m_drivetrainSubsystem::getGyroscopeRotation,
//                m_drivetrainSubsystem::setStates,
//                m_drivetrainSubsystem
//                        );
//





// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
       HashMap<String, Command> eventMap = new HashMap<>();
//        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
//        eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_drivetrainSubsystem::getPos, // Pose2d supplier
                m_drivetrainSubsystem::ResetPos, // Pose2d consumer, used to reset odometry at the beginning of auto
                m_drivetrainSubsystem.get_kinematics(), // SwerveDriveKinematics
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                m_drivetrainSubsystem::setStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                m_drivetrainSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command fullAuto = autoBuilder.fullAuto(Path);

        return new SequentialCommandGroup(
                fullAuto
        );

    }
}

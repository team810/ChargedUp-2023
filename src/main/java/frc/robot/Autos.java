package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.*;

import java.util.HashMap;

public class Autos {
    private Drivetrain m_drivetrain;
    private Intake m_intake; 
    private Conveyor m_conveyor;
    private Arm m_arm;
    private Gripper m_gripper;
    private SwerveAutoBuilder m_AUTO_BUILDER;
    
    // Kinematics is the position of the modules on the chasis
    private final SwerveDriveKinematics m_kinematics;

    private final Command paths[] =
    {
        genPath("path")
    };


    public Autos(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Arm arm, Gripper gripper) {
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_conveyor = conveyor;
        m_arm = arm;
        m_gripper = gripper;
    
        addMethods();

        m_kinematics = new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                    DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Front right
                    new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                    -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back left
                    new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                    DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back right
                    new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                    -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // Auto Variables
        m_AUTO_BUILDER = new SwerveAutoBuilder(
        m_drivetrain::getPose,
        m_drivetrain::resetPose,
        this.m_kinematics,
        DrivetrainConstants.Auto.XY_CONSTANTS,
        DrivetrainConstants.Auto.THETA_CONSTANTS,
        m_drivetrain::setStates,
        this.eventMap,
        false,
        m_drivetrain);


    }

    // Auto Variables
    // This contains the methods we run during auto
    private final HashMap<String, Command> eventMap = new HashMap<>();

    public void addMethods() {

        // intake methodsWADWQWADSXCX6Y
        // eventMap.put("Actuate Intake", new InstantCommand(m_intake::actuateIntake));
        eventMap.put("Intake In", new InstantCommand(() -> m_intake.runIntake(1)));
        eventMap.put("Intake Out", new InstantCommand(() -> m_intake.runIntake(-1)));
        eventMap.put("Stop Intake", new InstantCommand(() -> m_intake.runIntake(0)));

        // conveyor methods
        eventMap.put("Conveyor In", new InstantCommand(() -> m_conveyor.runConveyor(1)));
        eventMap.put("Conveyor Out", new InstantCommand(() -> m_conveyor.runConveyor(-1)));
        eventMap.put("Stop Conveyor", new InstantCommand(() -> m_conveyor.runConveyor(0)));
        eventMap.put("Run Conveyor + Detect Color", new InstantCommand(m_conveyor::runConveyorWithColor));

        // arm methods
        eventMap.put("Rest Arm", new InstantCommand(m_arm::rest));
        eventMap.put("Set Arm to Middle Goal", new InstantCommand(m_arm::middleGoalCone));
        eventMap.put("Set Arm to High Goal", new InstantCommand(m_arm::highGoalCone));

        // gripper methods
        eventMap.put("Rest Gripper", new InstantCommand(m_gripper::rest));
        eventMap.put("Grip Cone", new InstantCommand(m_gripper::gripCone));
        eventMap.put("Grip Cube", new InstantCommand(m_gripper::gripCube));
    }

    public Command genPath(String path)
    {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrain.lockWheels()),
                m_AUTO_BUILDER.fullAuto(PathPlanner.loadPathGroup(path, Constants.DrivetrainConstants.Auto.PATH_CONSTRAINTS)),
                new InstantCommand(() -> m_drivetrain.unlockWheels())
        );
    }

    public Command getForward()
    {
        String path = "paths";
        return null;
    }

    // Kinematics is the position of the modules on the chasis
    public SwerveDriveKinematics getKinematics()
    {
        return m_kinematics;
    }
}

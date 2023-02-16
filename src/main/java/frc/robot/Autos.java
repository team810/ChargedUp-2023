package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

public class Autos extends CommandBase {
    private Drivetrain m_drivetrain;
    private Intake m_intake; 
    private Conveyor m_conveyor;
    private Arm m_arm;
    private Gripper m_gripper;
    private SwerveAutoBuilder m_AUTO_BUILDER;

    // Kinematics is the position of the modules on the chasis
    private final SwerveDriveKinematics m_kinematics;


    public Autos(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Arm arm, Gripper gripper) {
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_conveyor = conveyor;
        m_arm = arm;
        m_gripper = gripper;
        addRequirements(m_drivetrain, m_intake, m_conveyor, m_arm, m_gripper);
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

        // intake methods
        eventMap.put("Actuate Intake", new InstantCommand(m_intake::actuateIntake));
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
        eventMap.put("Set Arm to Middle Goal", new InstantCommand(m_arm::middleGoal));
        eventMap.put("Set Arm to High Goal", new InstantCommand(m_arm::highGoal));

        // gripper methods
        eventMap.put("Rest Gripper", new InstantCommand(m_gripper::rest));
        eventMap.put("Grip Cone", new InstantCommand(m_gripper::gripCone));
        eventMap.put("Grip Cube", new InstantCommand(m_gripper::gripCube));
    }

    // Kinematics is the position of the modules on the chasis

    public SwerveDriveKinematics getKinematics()
    {
        return m_kinematics;
    }
}

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.*;

import java.util.HashMap;

public class Autos {
    private final Drivetrain m_drivetrain;
    private final Intake m_intake;
    private final Conveyor m_conveyor;
    private final Arm m_arm;
    private final Gripper m_gripper;
    private final Limelight limelight;
    private final ScoreCommand score;
    private final SwerveAutoBuilder m_AUTO_BUILDER;



    public Autos(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Arm arm, Gripper gripper, Limelight limelight) {
//        PathPlannerServer.startServer(5811);
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_conveyor = conveyor;
        m_arm = arm;
        m_gripper = gripper;
        this.limelight = limelight;

        score = new ScoreCommand(arm, drivetrain, gripper, limelight, conveyor,2);

        addMethods();


        // Auto Variables
        m_AUTO_BUILDER = new SwerveAutoBuilder(
            m_drivetrain::getPose,
            m_drivetrain::resetPose,
            DrivetrainConstants.KINEMATICS,
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
        eventMap.put("Intake In", new InstantCommand(m_intake::in));
        eventMap.put("Intake Out", new InstantCommand(m_intake::out));
        eventMap.put("Stop Intake", new InstantCommand(m_intake::stopIntake));

        eventMap.put("Score", score);
    }

    public Command genPath(String path)
    {
        return m_AUTO_BUILDER.fullAuto(PathPlanner.loadPathGroup(path, Constants.DrivetrainConstants.Auto.PATH_CONSTRAINTS));
    }
}

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Autos extends CommandBase {
    private Drivetrain m_drivetrain;
    private final SwerveAutoBuilder AUTO_BUILDER;


    public Autos(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        AUTO_BUILDER = new SwerveAutoBuilder(
                m_drivetrain::getPose,
                m_drivetrain::resetPose,
                Constants.DrivetrainConstants.KINEMATICS,
                Constants.DrivetrainConstants.Auto.XY_CONSTANTS,
                Constants.DrivetrainConstants.Auto.THETA_CONSTANTS,
                m_drivetrain::setStates,
                this.eventMap,
                false,
                m_drivetrain);

        addRequirements(drivetrain);
    }

    // Auto Variables
    // This contains the methods we run during auto
    private final HashMap<String, Command> eventMap = new HashMap<>();

    // Auto Commands
    public Command forward() {
        return this.AUTO_BUILDER.fullAuto(PathPlanner.loadPathGroup("ForwardWithRot",
                new PathConstraints(4, 3)));
    }
}

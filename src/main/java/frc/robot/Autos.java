package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class Autos extends CommandBase {
    private Drivetrain m_drivetrain;

    public Autos(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Kinematics is the position of the modules on the chasis
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
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

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    // Auto Variables
    private final SwerveAutoBuilder m_AUTO_BUILDER = new SwerveAutoBuilder(
            m_drivetrain::getPose,
            m_drivetrain::resetPose,
            this.m_kinematics,
            DrivetrainConstants.Auto.XY_CONSTANTS,
            DrivetrainConstants.Auto.THETA_CONSTANTS,
            m_drivetrain::setStates,
            this.eventMap,
            false,
            m_drivetrain);

    // This contains the methods we run during auto
    private final HashMap<String, Command> eventMap = new HashMap<>();

    // Auto Commands
    public Command forward() {
        return this.m_AUTO_BUILDER.fullAuto(PathPlanner.loadPathGroup("ForwardWithRot",
                new PathConstraints(4, 3)));
    }
}

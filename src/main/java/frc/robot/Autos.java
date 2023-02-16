package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

public class Autos extends CommandBase {
    private Drivetrain m_drivetrain;
    
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
}

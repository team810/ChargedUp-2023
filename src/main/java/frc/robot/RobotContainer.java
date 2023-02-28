package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.StartupCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
        //FIXME Need Bot
        private final Drivetrain m_drive = new Drivetrain();
        private final Arm m_arm = new Arm();
        private final Intake m_intake = new Intake();
        private final Gripper m_gripper = new Gripper();
        private final Conveyor m_conveyor = new Conveyor();
        private final Limelight m_lime = new Limelight();
        private final Autos autos = new Autos(m_drive, m_intake, m_conveyor, m_arm, m_gripper, m_lime);

        // private final boolean ScoringCommandActive = false;

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
                                                                DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                                                                * 0.25)));

                configureButtonBindings();
        }

        private void configureButtonBindings() {
                // Driving gamepad
                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(12)).onTrue(
                                new InstantCommand(m_drive::zeroGyroscope));

                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(1)).whileTrue(
                                new ParallelCommandGroup(
                                                new StartEndCommand(
                                                                m_intake::runIntakeReversed,
                                                                m_intake::stopIntake,
                                                                m_intake),
                                                new StartEndCommand(
                                                                () -> {
                                                                        m_conveyor.setReversed(true);
                                                                        m_conveyor.setEnabled(true);
                                                                },
                                                                () -> {
                                                                        m_conveyor.setReversed(false);
                                                                        m_conveyor.setEnabled(false);
                                                                },
                                                                m_conveyor)));
                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(4)).whileTrue(
                                new StartEndCommand(
                                                m_intake::runIntake,
                                                m_intake::stopIntake,
                                                m_intake).alongWith(
                                                                new StartEndCommand(
                                                                                () -> {
                                                                                        m_conveyor.setReversed(false);
                                                                                        m_conveyor.setEnabled(true);
                                                                                },
                                                                                () -> {
                                                                                        m_conveyor.setReversed(false);
                                                                                        m_conveyor.setEnabled(false);
                                                                                },
                                                                                m_conveyor)));

                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(2)).onTrue(
                                new InstantCommand(m_intake::out));

                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(3)).onTrue(
                                new InstantCommand(m_intake::in));

                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(5)).toggleOnTrue(
                                new ScoreCommand(m_arm, m_drive, m_gripper, m_lime, m_conveyor, 2));
                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(6)).toggleOnTrue(
                                new ScoreCommand(m_arm, m_drive, m_gripper, m_lime, m_conveyor, 3));

                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(9)).onTrue(
                                new InstantCommand(m_gripper::closeGripper));

        }

        public void teleopInit() {
                CommandScheduler.getInstance().schedule(new StartupCommands(m_gripper));
        }

        public Command getAutonomousCommand() {
                // TODO: Set new path before each match
                return autos.genPath("Charge Station");
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
}

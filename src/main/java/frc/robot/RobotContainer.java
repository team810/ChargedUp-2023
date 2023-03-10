package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GripperSetpoint;
import frc.robot.commands.RaiseArmCommand;
import frc.robot.commands.StartupCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.HardStopSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
        private final Drivetrain m_drive = new Drivetrain();
        private final Arm m_arm = new Arm();
        private final Intake m_intake = new Intake();
        private final Conveyor m_conveyor = new Conveyor();
        private final Gripper m_gripper = new Gripper(m_conveyor);
        private final Limelight m_lime = new Limelight();
        private final HardStopSubsystem m_hardStop = new HardStopSubsystem();
        // private final Autos autos = new Autos(m_drive, m_intake, m_conveyor, m_arm,
        // m_gripper, m_lime);

        // private final boolean ScoringCommandActive = false;

        public RobotContainer() {
                m_lime.setMode("Reflective Tape");

                m_drive.setDefaultCommand(new DefaultDriveCommand(
                                m_drive,
                                () -> -modifyAxis(OIConstants.DRIVE_GAMEPAD.getLeftY() *
                                                DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND),
                                () -> -modifyAxis(OIConstants.DRIVE_GAMEPAD.getLeftX() *
                                                DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND),
                                () -> -modifyAxis(
                                                OIConstants.DRIVE_GAMEPAD.getRightX() *
                                                                DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

                m_gripper.setDefaultCommand(
                                new GripperSetpoint(m_gripper, () -> OIConstants.SECONDARY_GAMEPAD.getRawAxis(1)));
                m_conveyor.setDefaultCommand(
                                new ConveyorCommand(m_conveyor, () -> OIConstants.SECONDARY_GAMEPAD.getRawAxis(4)));

                configureButtonBindings();
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

        private void configureButtonBindings() {
                //Testing gripper
                new Trigger(()-> OIConstants.SECONDARY_GAMEPAD.getAButton()).onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> m_gripper.gripPiece(true)),
                                new WaitCommand(1),
                                new InstantCommand(()-> m_gripper.openGripper(true))
                ));
                
                // Primary
                //Zero gyro
                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(12)).onTrue(
                                new InstantCommand(m_drive::zeroGyroscope));
                //Intake back
                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getAButton()).whileTrue(
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
                // Intake forward
                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getYButton()).whileTrue(
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

                //Secondary
                // Intake Reverse
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getAButton()).whileTrue(
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

                // Intake forward
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getYButton()).whileTrue(
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

                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getBButton()).onTrue(
                                new InstantCommand(() -> m_hardStop.in()));

                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getXButton()).toggleOnTrue(
                                new InstantCommand(() -> m_hardStop.out()));

                //FIXME Why do we do this here?
                // new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getBButton()).onTrue(
                //                 new RepeatCommand(new InstantCommand(
                //                                 () -> m_arm.setExtenderSetpoint(m_arm.getExtenderSetpoint() + .5))));

                // Medium score
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(6))
                                .toggleOnTrue(new RaiseArmCommand(m_arm, 2, 1));
                // High score
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(12))
                                .toggleOnTrue(new RaiseArmCommand(m_arm, 3, 1));

                // Run after scoring
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(5)).whileTrue(
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> m_arm.restExtender()),
                                                                new WaitCommand(.75),
                                                                new InstantCommand(() -> m_arm.restPivot())),
                                                new StartupCommands(m_gripper)));
        }

        public void teleopInit() {
                CommandScheduler.getInstance().schedule(new StartupCommands(m_gripper));
        }

        public Command getAutonomousCommand() {
                //TODO: Change to auto path we want this match, get name from pathplanner
                // return autos.genPath("Rot Tunning");
                return null;
        }
}
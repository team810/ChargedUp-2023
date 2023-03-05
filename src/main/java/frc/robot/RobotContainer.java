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
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GripperSetpoint;
import frc.robot.commands.RaiseArmCommand;
import frc.robot.commands.StartupCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
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
                                new GripperSetpoint(m_gripper, () -> Math.pow(-OIConstants.SECONDARY_GAMEPAD.getLeftY(), 3)));
                // m_arm.setDefaultCommand(
                //         new ManualPivot(m_arm, m_intake, ()-> -OIConstants.SECONDARY_GAMEPAD.getRawAxis(4))
                // );

                configureButtonBindings();
        }

        private void configureButtonBindings() {
                // Primary
                //Zero gyro
                new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(5)).onTrue(
                                new InstantCommand(m_drive::zeroGyroscope));
                //Speed limit
                new Trigger(()-> OIConstants.DRIVE_GAMEPAD.getRightBumper()).toggleOnTrue(
                        new StartEndCommand(()-> m_drive.slow(), ()-> m_drive.normal(), m_drive)
                );

                //Secondary
                //Extender
                // new Trigger(()-> OIConstants.SECONDARY_GAMEPAD.getRawButton(9)).onTrue(
                //         new StartEndCommand(()-> m_arm.runExtender(.5),()-> m_arm.runExtender(0), m_arm).andThen(
                //                 new InstantCommand(m_arm::usePID)
                //         )
                // );
                // new Trigger(()-> OIConstants.SECONDARY_GAMEPAD.getRawButton(14)).onTrue(
                //         new StartEndCommand(()-> m_arm.runExtender(-.5),()-> m_arm.runExtender(0), m_arm).andThen(
                //                 new InstantCommand(m_arm::usePID))
                // );
                
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
                // Intake toggle
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getXButton()).onTrue(
                                new InstantCommand(m_intake::in));
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getBButton()).onTrue(
                        new InstantCommand(m_intake::out));        

                // Medium score
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(6)).toggleOnTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> m_intake.in()),
                                                new RaiseArmCommand(m_arm, 2, 2)));
                // High score
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(12)).toggleOnTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> m_intake.in()),
                                                new RaiseArmCommand(m_arm, 3, 2))

                );

                // Run after scoring
                new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(5)).whileTrue(
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> m_arm.restExtender()),
                                                                new WaitCommand(.75),
                                                                new InstantCommand(() -> m_arm.restPivot())),
                                                new StartupCommands(m_gripper)));

                // //Conveyor 9 = hamburger
                // new Trigger(()-> OIConstants.SECONDARY_GAMEPAD.getRawButton(14)).whileTrue(
                //         new StartEndCommand(
                //                 () -> {
                //                         m_conveyor.setEnabled(true);
                //                         m_conveyor.setReversed(false);
                //                         m_conveyor.setOvorride(true);
                //                 }, 
                //                 () -> 
                //                 {
                //                         m_conveyor.setEnabled(true);
                //                         m_conveyor.setReversed(false);
                //                         m_conveyor.setOvorride(true);
                //                 }, m_conveyor)
                // );
                // //14 = cube
                // new Trigger(()-> OIConstants.SECONDARY_GAMEPAD.getRawButton(14)).whileTrue(
                //         new StartEndCommand(
                //                 () -> {
                //                         m_conveyor.setEnabled(true);
                //                         m_conveyor.setReversed(true);
                //                         m_conveyor.setOvorride(true);
                //                 }, 
                //                 () -> 
                //                 {
                //                         m_conveyor.setEnabled(true);
                //                         m_conveyor.setReversed(true);
                //                         m_conveyor.setOvorride(true);
                //                 }, m_conveyor)
                // );

                // Close Gripper
                // new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getPOV() == 0).whileTrue(
                //                 new InstantCommand(m_gripper::gripCone));
                // Open Gripper
                // new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getPOV() == 180).whileTrue(
                //                 new InstantCommand(m_gripper::openGripper));
        }

        public void teleopInit() {
                CommandScheduler.getInstance().schedule(new StartupCommands(m_gripper));
        }

        public Command getAutonomousCommand() {

                // return autos.genPath("Red 1");
                return null;
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
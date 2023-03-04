package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.StartupCommands;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final Drivetrain m_drive = new Drivetrain();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();
    private final Gripper m_gripper = new Gripper();
    private final Conveyor m_conveyor = new Conveyor();
    private final Limelight m_lime = new Limelight();
    private final Autos autos = new Autos(m_drive, m_intake, m_conveyor, m_arm, m_gripper, m_lime);

    private final boolean ScoringCommandActive = false;

    public RobotContainer() {
        m_lime.setMode("Reflective Tape");

//        // Set up the default command for the drivetrain.
        m_drive.setDefaultCommand(new DefaultDriveCommand(
                m_drive,
                () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getRawAxis(1) *
                        DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.25),
                () -> modifyAxis(OIConstants.DRIVE_GAMEPAD.getRawAxis(0) *
                        DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.25),
                () -> -modifyAxis(
                        OIConstants.DRIVE_GAMEPAD.getRawAxis(3) *
                                DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.25),
                OIConstants.DRIVE_GAMEPAD::getPOV
        ));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Driving gamepad
        new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(12)).onTrue(
                new InstantCommand(m_drive::zeroGyroscope)
        );

        new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(1)).whileTrue(
                new ParallelCommandGroup(
                        new StartEndCommand(
                                m_intake::runIntakeReversed,
                                m_intake::stopIntake,
                                m_intake
                        ),
                        new StartEndCommand(
                                () -> {
                                    m_conveyor.setReversed(true);
                                    m_conveyor.setEnabled(true);
                                },
                                () -> {
                                    m_conveyor.setReversed(false);
                                    m_conveyor.setEnabled(false);
                                },
                                m_conveyor
                        )
                )
        );

        new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(4)).whileTrue(
                new StartEndCommand(
                        m_intake::runIntake,
                        m_intake::stopIntake,
                        m_intake
                ).alongWith(
                        new StartEndCommand(
                                () -> {
                                    m_conveyor.setReversed(false);
                                    m_conveyor.setEnabled(true);
                                },
                                () -> {
                                    m_conveyor.setReversed(false);
                                    m_conveyor.setEnabled(false);
                                },
                                m_conveyor
                        )));

        new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(2)).onTrue(
                new InstantCommand(m_intake::out)
        );
        new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(3)).onTrue(
                new InstantCommand(m_intake::in)
        );



        new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(5)).toggleOnTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_intake.in()),
                        new ScoreCommand(m_arm, m_drive, m_gripper, m_lime, m_conveyor, m_intake,2, 1)
                )
        );
        new Trigger(() -> OIConstants.SECONDARY_GAMEPAD.getRawButton(6)).toggleOnTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_intake.in()),
                        new ScoreCommand(m_arm, m_drive, m_gripper, m_lime, m_conveyor, m_intake,3, 1)
                )

        );

        // Reset pos
        new Trigger(() -> OIConstants.DRIVE_GAMEPAD.getRawButton(6)).whileTrue(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> m_arm.restExtender()),
                                new WaitCommand(.75),
                                new InstantCommand(() -> m_arm.restPivot())
                        ),
                        new StartupCommands(m_gripper)
                )
        );


        // The pid controller needs to be off for this
        // Manual
        //Extender

//        new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(5)).whileTrue(
//                new StartEndCommand(()-> m_arm.runExtender(.4), ()-> m_arm.runExtender(0), m_arm)
//        );
//        new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(6)).whileTrue(
//                new StartEndCommand(()-> m_arm.runExtender(-.4), ()-> m_arm.runExtender(0), m_arm)
//        );
//
//        new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(1)).whileTrue(
//                new StartEndCommand(()-> m_arm.runPivot(.2), ()-> m_arm.runPivot(0), m_arm)
//        );
//        new Trigger(()->OIConstants.DRIVE_GAMEPAD.getRawButton(4)).whileTrue(
//                new StartEndCommand(()-> m_arm.runPivot(-.2), ()-> m_arm.runPivot(0), m_arm)
//        );
    }

    public void teleopInit()
    {
        CommandScheduler.getInstance().schedule(new StartupCommands(m_gripper));

    }

    public Command getAutonomousCommand() {

        return autos.genPath("Rot Tunning");
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

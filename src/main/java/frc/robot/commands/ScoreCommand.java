package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Limelight;

public class ScoreCommand extends SequentialCommandGroup {
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final Conveyor conveyor;
    private int target;
    private final ToTargetCommand toTarget;
    private int gamePiece;

    public ScoreCommand(Arm arm, Drivetrain drivetrain, Gripper gripper, Limelight limelight,

                        Conveyor conveyor, Intake intake, int target, int targetGrid) {
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;

        this.toTarget = new ToTargetCommand(conveyor, drivetrain, limelight);

        this.target = target;
        addCommands(
                new InstantCommand(() -> intake.in()),
                new InstantCommand(() -> intake.setScoring(true)),
                new InstantCommand(() -> arm.setExtenderSetpoint(-1)),
                toTarget,
                new InstantCommand(gripper::openGripper),
                new ConveyorCommand(conveyor), // This is the command so the conveyor positions the game piece.
                new InstantCommand(() -> {
                    gamePiece = conveyor.getGamePiece();
                }), // this sets the game piece after the conveyor runs
                gripGamePiece(),
                new WaitCommand(.25),
                new InstantCommand(() -> arm.setExtenderSetpoint(-3.5)),
                new RaiseArmCommand(arm, target, targetGrid),
                new WaitCommand(1.1),
                releaseGrip(),
                new WaitCommand(.5),
                new InstantCommand(arm::restExtender),
                new InstantCommand(gripper::closeGripper),
                new WaitCommand(.8),
                new InstantCommand(arm::restPivot),
                new WaitCommand(2),
                new InstantCommand(() -> intake.setScoring(false)),
                new InstantCommand(gripper::openGripper)
        );

        addRequirements(this.arm, this.drivetrain, this.gripper, this.limelight, this.conveyor);
    }

    // private Command setTarget() {
    //     return new InstantCommand(() -> {
    //         if (conveyor.getGamePiece() == 1) { // cone
    //             // target = 3;
    //             System.out.println("Hello World\n");
    //         } else if (conveyor.getGamePiece() == 2) { // cube
    //             target = 1;
    //         } else {
    //             System.out.println("NO target");
    //         }
    //     });
    // }

    private Command gripGamePiece() {
        return new InstantCommand(() -> {
            if (conveyor.getGamePiece() == 1) {
                gripper.gripCone();
            } else if (conveyor.getGamePiece() == 2) {
                gripper.gripCube();
            }
            else{
            gripper.gripCube(); // this is not a good default state
            }
        });
    }

    public Command releaseGrip() {
        return new InstantCommand(gripper::openGripper);
    }
}


package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ScoreCommand extends SequentialCommandGroup {
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final Conveyor conveyor;
    private int target;
    private ToTargetCommand toTarget;
    public ScoreCommand(Arm arm, Drivetrain drivetrain, Gripper gripper, Limelight limelight,
                        Conveyor conveyor, int target) {
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;

        this.toTarget = new ToTargetCommand(conveyor, drivetrain, limelight);

        this.target = target;
        addCommands(
                toTarget,
                new InstantCommand(() -> drivetrain.drive(new ChassisSpeeds(0,0,0))), // Just making sure that the robot is not moving at all
                new InstantCommand(() -> conveyor.setScoring(true)),
                new InstantCommand(() -> gripper.rest()),
                new InstantCommand(() -> conveyor.conveyorMotor.set(.3)),
                new WaitCommand(1),
                new InstantCommand(() -> conveyor.conveyorMotor.set(0)),
                new InstantCommand(() -> conveyor.setScoring(false)),
                setTarget(),
                gripGamePiece(),
                armToGoal(),
                new WaitCommand(3),
                extenderToGoal(),
                new WaitCommand(2),
                releaseGrip(),
                new WaitCommand(2),
                new InstantCommand(() -> arm.restExtender()),
                new InstantCommand(() -> gripper.gripCone()),
                new WaitCommand(1.5),
                new InstantCommand(() -> arm.restPivot()),
                new WaitCommand(2),
                new InstantCommand(() -> gripper.deffultstate()),
                new InstantCommand(() -> System.out.println("finished"))
        );
        addRequirements(this.arm, this.drivetrain, this.gripper, this.limelight, this.conveyor);
    }


    private Command setTarget() {
        return new InstantCommand(() -> {
            if (conveyor.getGamePiece() == 1) { // cone
//                target = 3;
                System.out.println("Hello World\n");
            } else if (conveyor.getGamePiece() == 2) { // cube
                target = 1;
            } else {
                System.out.println("NO target");
            }
        });
    }

    private Command gripGamePiece()
    {
        return new InstantCommand(() ->
        {
            gripper.gripCone();
        });
    }

    public Command releaseGrip()
    {
        return new InstantCommand(() -> {
            gripper.rest();
        });
    }

    private Command armToGoal()
    {
        return new InstantCommand(() -> {
            switch (target) {
                case 1:
                    arm.setPivotSetpoint(-9.5);
                    break;
                case 2:
                    arm.setPivotSetpoint(-35.5);
                    break;
                case 3:
                    arm.setPivotSetpoint(-40);
                    break;
                default:
                    System.out.println("how did you get here");
                    break;
            }
        });
    }
    private Command extenderToGoal()
    {
        return new InstantCommand(() ->
        {
            switch (target)
            {
                case 1:
                    arm.setExtenderSetpoint(-2.5);
                    break;
                case 2:
                    arm.setExtenderSetpoint(5);
                    break;
                case 3:
                    arm.setExtenderSetpoint(20.5);
                    break;
                default:
                    System.out.println("how did you get here");
                    break;
            }
        });
    }

}

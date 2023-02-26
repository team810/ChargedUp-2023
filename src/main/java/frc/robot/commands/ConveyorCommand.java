package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;


public class ConveyorCommand extends SequentialCommandGroup {
    private final Conveyor conveyor;

    private final double CONE_TIME = .5;
    private final double CUBE_TIME = .5;

    private final double CONVEYOR_SPEED = .1;

    double waitTime = 0;
    public ConveyorCommand(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(this.conveyor);

        addCommands(
                new InstantCommand(() -> System.out.println("Hello World")),
                new InstantCommand(() -> conveyor.setScoring(true)),
                new InstantCommand(() -> conveyor.setEnabled(false)),
                new InstantCommand(() -> conveyor.conveyorMotor.set(.2)),
                new InstantCommand(() -> {
                    if (conveyor.getGamePiece() == 1)
                    {
                        waitTime = CONE_TIME;
                    } else if (conveyor.getGamePiece() == 2) {
                        waitTime = CUBE_TIME;
                    }else{
                        waitTime = 2;
                        System.out.println("ERROR");
                    }
                }),
                new WaitCommand(waitTime),
                new InstantCommand(() -> conveyor.setScoring(false))
        );
    }
}

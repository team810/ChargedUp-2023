package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;


public class ConveyorCommand extends SequentialCommandGroup {
    private final Conveyor conveyor;

    private final double CONE_TIME = .5; // FIXME times need to be worked out
    private final double CUBE_TIME = .5;

    private final double CONVEYOR_SPEED = .15;
    double waitTime = 0;

    public ConveyorCommand(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(this.conveyor);

        addCommands(
                init(),
                new WaitCommand(waitTime),
                end()
        );
    }

    public Command init()
    {
        return new InstantCommand(() ->
            {
                System.out.println("Conveyor Command Started");

                conveyor.setScoring(true);
                conveyor.setEnabled(false);

                conveyor.conveyorMotor.set(CONVEYOR_SPEED);

                if (conveyor.getGamePiece() == 1)
                {
                    waitTime = CONE_TIME;
                } else if (conveyor.getGamePiece() == 2) {
                    waitTime = CUBE_TIME;
                }else{
                    waitTime = CONE_TIME;
                }
            }
        );
    }

    public Command end()
    {
        return new InstantCommand(() ->
        {
            conveyor.conveyorMotor.set(0);

            conveyor.setScoring(false);
            conveyor.setEnabled(false);
        });
    }
}

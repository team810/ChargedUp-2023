package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ScoreCommand extends CommandBase {
    private final Arm arm;
    private final ColorSensor colorSensor;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final Conveyor conveyor;
    private boolean finished;

    // FIXME Lime light implementation
    public ScoreCommand(Arm arm, ColorSensor colorSensor, Drivetrain drivetrain, Gripper gripper, Limelight limelight,
            Conveyor conveyor, int target[]) {
        this.arm = arm;
        this.colorSensor = colorSensor;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;

        // this.turnToTarget = new TurnToTarget(drivetrain, limelight);

        this.finished = false;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.colorSensor, this.drivetrain, this.gripper, this.limelight, this.conveyor);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // set arm to hight

        new WaitCommand(2);

        gripper.rest();

        arm.rest();

        finished = true;
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run
        // execute()
        return finished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

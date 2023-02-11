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
    private final TurnToTarget turnToTarget;
    private final ToArmCommand toArm;
    private boolean finished;
    private int[] target = {0,0};

    // FIXME Lime light implementation
    public ScoreCommand(Arm arm, ColorSensor colorSensor, Drivetrain drivetrain, Gripper gripper, Limelight limelight, Conveyor conveyor, int target[]) {
        this.arm = arm;
        this.colorSensor = colorSensor;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;

        this.turnToTarget = new TurnToTarget(drivetrain, limelight);

        this.toArm = new ToArmCommand(arm,conveyor,gripper);

        this.finished = false;

        this.target = target;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.colorSensor, this.drivetrain, this.gripper, this.limelight, this.conveyor);
    }

    @Override
    public void initialize() {
        toArm.initialize();
    }

    @Override
    public void execute() {
        while(!toArm.isFinished())
        {
            toArm.execute();
        }

        arm.setTarget(target);

        new WaitCommand(2);

        gripper.rest();

        arm.setTarget(new int[]{0, 0});

        finished = true;
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return finished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

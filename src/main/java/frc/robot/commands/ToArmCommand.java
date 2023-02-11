package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Gripper;


public class ToArmCommand extends CommandBase {
    private final Arm arm;
    private final Conveyor conveyor;
    private final Gripper gripper;

    public ToArmCommand(Arm arm, Conveyor conveyor, Gripper gripper) {
        this.arm = arm;
        this.conveyor = conveyor;
        this.gripper = gripper;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.conveyor, this.gripper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

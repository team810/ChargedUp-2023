package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Limelight;

public class ScoreCommand extends CommandBase {
    private final Arm arm;
    private final ColorSensor colorSensor;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final TurnToTarget turnToTarget;
    private final SquareToTargetCommand squareToTarget;

    public ScoreCommand(Arm arm, ColorSensor colorSensor, Drivetrain drivetrain, Gripper gripper, Limelight limelight) {
        this.arm = arm;
        this.colorSensor = colorSensor;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.turnToTarget = new TurnToTarget(drivetrain, limelight);
        this.squareToTarget = new SquareToTargetCommand(drivetrain, limelight, 0);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.colorSensor, this.drivetrain, this.gripper, this.limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run
        // execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

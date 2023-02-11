package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class ScoreCommand extends CommandBase {
    private final Arm arm;
    private final ColorSensor colorSensor;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final Conveyor conveyor;
    private final TurnToTarget turnToTarget;
    private final SquareToTargetCommand squareToTarget;
    private final ToArmCommand toArm;
    private boolean finished;
    private final int target = 0;
    public ScoreCommand(Arm arm, ColorSensor colorSensor, Drivetrain drivetrain, Gripper gripper, Limelight limelight, Conveyor conveyor) {
        this.arm = arm;
        this.colorSensor = colorSensor;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;

        this.turnToTarget = new TurnToTarget(drivetrain, limelight);
        this.squareToTarget = new SquareToTargetCommand(drivetrain, limelight,0);
        this.toArm = new ToArmCommand(arm,conveyor,gripper);

        this.finished = false;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.colorSensor, this.drivetrain, this.gripper, this.limelight, this.conveyor);
    }

    @Override
    public void initialize() {
        squareToTarget.initialize();
        toArm.initialize();
    }

    @Override
    public void execute() {

        squareToTarget.execute();
        while(!squareToTarget.isFinished()) {squareToTarget.execute();}

        toArm.execute();
        while(!toArm.isFinished()) {toArm.execute();}


        arm.setArmTargetLength(Constants.ArmConstants.ARM_HIGHT_POINTS[target]);
        arm.setArmHight(Constants.ArmConstants.EXTENDER_LENGTH_POINTS[target]);

        new WaitCommand(1.5);
        gripper.rest();

        arm.setArmTargetLength(Constants.ArmConstants.ARM_HIGHT_POINTS[0]);
        arm.setArmHight(Constants.ArmConstants.EXTENDER_LENGTH_POINTS[0]);

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

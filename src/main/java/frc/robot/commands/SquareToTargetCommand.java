package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;


public class SquareToTargetCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final PIDController xController = Constants.DrivetrainConstants.Auto.XY_CONTTROLLER;
    private final PIDController yController = Constants.DrivetrainConstants.Auto.XY_CONTTROLLER;
    private final PIDController thetaController = Constants.DrivetrainConstants.Auto.THEATA_CONTTROLLER;
    private Pose3d targetPos;
    private Transform3d toTarget;
    private Pose3d currentPos;
    private final Transform3d distanceFromTarget = new Transform3d(new Translation3d(-.5,0,0),new Rotation3d());
    double error_amount = .05;
    public SquareToTargetCommand(Drivetrain drivetrain, Limelight limelight, int object) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        currentPos = new Pose3d(drivetrain.getPose());

        toTarget = limelight.getBestTarget().getBestCameraToTarget();

        targetPos = currentPos;
        targetPos.transformBy(toTarget);

        targetPos.transformBy(basedOnTargetTransform(object));

        targetPos.transformBy(distanceFromTarget);// makes it so the target pos is not on the actual target



        addRequirements(this.drivetrain, this.limelight);
    }

    private Transform3d basedOnTargetTransform(int object) // FIXME needs to be adjusted based on cones and cubes
    {
        return new Transform3d();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ChassisSpeeds speeds;

        double x_speed, y_speed, z_speed;

        x_speed = xController.calculate(drivetrain.getPose().getX(), targetPos.getX());
        y_speed = yController.calculate(drivetrain.getPose().getY(), targetPos.getY());
        z_speed = thetaController.calculate(drivetrain.getPose().getRotation().getRadians(), targetPos.getRotation().getAngle());

        speeds = new ChassisSpeeds(x_speed, y_speed, z_speed);

        drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {

        return xController.getPositionError() < error_amount &&
                yController.getPositionError() < error_amount &&
                thetaController.getPositionError() < error_amount;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

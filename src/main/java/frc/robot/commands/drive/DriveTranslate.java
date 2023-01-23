package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveTranslate extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private Pose2d NewPos;

    private double TurnTime = 0;

    private double Time = 0;
    private double finalX = 0;
    private double finalY = 0;
    private double finalR = 0;
    private boolean finished = false;

    public DriveTranslate(DrivetrainSubsystem drivetrainSubsystem, Pose2d NewPos) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.NewPos = NewPos;
        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d OldPos = drivetrainSubsystem.GetPos();
        Transform2d transform = new Transform2d(OldPos, NewPos);
        double X, Y, R;
        X = transform.getX();
        Y = transform.getY();
        R = transform.getRotation().getRadians();

        Time = 0;
        if (X > Y)
        {
            Time = Constants.Auto.MaxSpeed / X;
        } else if (X < Y) {
            Time = Constants.Auto.MaxSpeed / Y;
        }else{
            Time = Constants.Auto.MaxSpeed / X;
        }
        TurnTime = R / Constants.Auto.TurningSpeed;

        X = X * (Time /Constants.Auto.MaxSpeed);
        Y = Y * (Time /Constants.Auto.MaxSpeed);
        R = R * (TurnTime /Constants.Auto.TurningSpeed);

        System.out.println(X);
        System.out.println(Y);
        System.out.println(R);
        System.out.println("Hi\n");

        finalX = X;
        finalY = Y;
        finalR = R;
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(new ChassisSpeeds(finalX, finalY, finalR));
        new WaitCommand(TurnTime);
        drivetrainSubsystem.drive(new ChassisSpeeds(finalX, finalY, 0));
        new WaitCommand(Time);
        drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
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

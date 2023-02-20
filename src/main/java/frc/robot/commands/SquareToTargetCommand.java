package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autos;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.function.Supplier;


public class SquareToTargetCommand extends SequentialCommandGroup {
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private Supplier<Integer> object;
    private Autos autos;

    public SquareToTargetCommand(Drivetrain drivetrain, Limelight limelight, Supplier<Integer> object, Autos auto) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.object = object;
        autos = auto;

        Init();

        addRequirements(this.drivetrain, this.limelight);
    }

    public void Init()
    {
        if (object.get() == 1) // Cone
        {
            System.out.println("Cone");
            limelight.setMode("Reflective Tape");
            addCommands(targetPosCone());

        } else if (object.get() == 2){ // Cube
            System.out.println("Cube");
            limelight.setMode("AprilTag");
            addCommands(targetPosCube());

        }else{
            System.out.println("VERY BAD SQUARE TO TARGET IS NOT LIKING LIFE");
        }


    }

    public Command targetPosCube()
    {
//        Transform3d temp = new Transform3d(new Translation3d(-.669,0,0), new Rotation3d()); // I do not know what the x and y will be

        Translation2d trans = limelight.getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d();
        trans.plus(new Translation2d(-669,0));

        return autos.gnerateCommand(
                PathPlanner.generatePath(
                        Constants.DrivetrainConstants.Auto.SCORE_CONSTRAINTS,
                        new PathPoint(trans, limelight.getBestTarget().getBestCameraToTarget().getRotation().toRotation2d()),
                        new PathPoint(new Translation2d(), new Rotation2d())
                )
        );
    }

    public Command targetPosCone()
    {
//        Transform3d temp = new Transform3d(new Translation3d(-.488,0,0), new Rotation3d()); // I do not know what the x and y will be

        Translation2d trans = limelight.getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d();
        trans.plus(new Translation2d(-.488, 0));

        return autos.gnerateCommand(
                PathPlanner.generatePath(
                        Constants.DrivetrainConstants.Auto.SCORE_CONSTRAINTS,
                        new PathPoint(trans, limelight.getBestTarget().getBestCameraToTarget().getRotation().toRotation2d()),
                        new PathPoint(new Translation2d(), new Rotation2d())
                )
        );
    }


}
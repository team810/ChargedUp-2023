package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final DoubleSupplier mDpad;

    private final PIDController rotController = new PIDController(15,0,0);
    private double setPoint = 0;
    private boolean dpadTurning = false;

    public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            DoubleSupplier dpad) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        mDpad = dpad;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement

        // this is for driving controls and setting the angle easily.
        double rot;
        if (mDpad.getAsDouble() == -1 || dpadTurning)
        {
            rot = m_rotationSupplier.getAsDouble();
        }else {
            setPoint = mDpad.getAsDouble();
            rot = rotController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), setPoint);

            rot = Math.min(rot, -0.75);
            rot = Math.max(rot, .75);
        }


        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        rot,
                        m_drivetrainSubsystem.getGyroscopeRotation()));

//        // m_drivetrainSubsystem.drive(
        //     new ChassisSpeeds(
        //         m_translationXSupplier.getAsDouble(),
        //         m_translationYSupplier.getAsDouble(),
        //         m_rotationSupplier.getAsDouble()
        //     )
        // );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

}

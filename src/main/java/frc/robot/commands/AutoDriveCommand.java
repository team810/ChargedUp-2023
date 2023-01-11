package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_translationXSupplier;
    private final double m_translationYSupplier;
    private final double m_rotationSupplier;

    public AutoDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               double translationXSupplier,
                               double translationYSupplier,
                               double rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier,
                        m_translationYSupplier,
                        m_rotationSupplier,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}

package com.swervedrivespecialties.swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public interface SwerveModule {
    double getDriveVelocity();

    void setDriveMotorIdleState(CANSparkMax.IdleMode idleState);
    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    RelativeEncoder getDriveEncoder();
}

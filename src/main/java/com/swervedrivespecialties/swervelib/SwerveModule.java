package com.swervedrivespecialties.swervelib;

import com.revrobotics.RelativeEncoder;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    RelativeEncoder getDriveEncoder();
}

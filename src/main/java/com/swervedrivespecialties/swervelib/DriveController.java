package com.swervedrivespecialties.swervelib;

import com.revrobotics.RelativeEncoder;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    RelativeEncoder getDriveEncoder();
}

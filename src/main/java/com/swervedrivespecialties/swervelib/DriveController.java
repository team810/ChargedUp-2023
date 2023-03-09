package com.swervedrivespecialties.swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public interface DriveController {
	void setReferenceVoltage(double voltage);

	double getStateVelocity();

	void setIdleMode(CANSparkMax.IdleMode idleMode);

	RelativeEncoder getDriveEncoder();
}

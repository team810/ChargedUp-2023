package com.swervedrivespecialties.swervelib;

import com.revrobotics.CANSparkMax;

public interface SteerController {
	double getReferenceAngle();

	void setReferenceAngle(double referenceAngleRadians);

	double getStateAngle();

	void setIleMode(CANSparkMax.IdleMode mIdleState);
}

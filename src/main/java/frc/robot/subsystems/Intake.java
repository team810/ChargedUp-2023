package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
	private final CANSparkMax leftIntakeMotor, rightIntakeMotor;

	public Intake() {
		leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
		rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

		leftIntakeMotor.restoreFactoryDefaults();
		rightIntakeMotor.restoreFactoryDefaults();

		leftIntakeMotor.setSmartCurrentLimit(20);
		rightIntakeMotor.setSmartCurrentLimit(20);

		leftIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}
	public void runIntake() {
		leftIntakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
		rightIntakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
	}

	public void stopIntake() {
		leftIntakeMotor.set(0);
		rightIntakeMotor.set(0);
	}

	public void runIntakeReversed() {
		leftIntakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
		rightIntakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
	}

}

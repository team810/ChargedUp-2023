package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
	private final CANSparkMax leftIntakeMotor, rightIntakeMotor;
	private final ShuffleboardLayout INTAKE_VALUES = IntakeConstants.INTAKE_VALUES;


	private final Boolean scoring;


	/**
	 * Creates a new Intake.
	 */
	public Intake() {
		leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
		rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

		scoring = false;
		shuffleboardInit();
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

	public void shuffleboardInit() {
		INTAKE_VALUES.addDouble("Velocity", () -> leftIntakeMotor.getEncoder().getVelocity());
	}

}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
	private final CANSparkMax extendingMotor, pivotMotor;
	private final PIDController extenderController, pivotController;
	private final AnalogInput potReading;
	private final ShuffleboardLayout PIVOT, EXTENDER;
	private boolean isManual;

	public double getExtenderSetpoint() {
		return extenderSetpoint;
	}

	private double extenderSetpoint;
	private double pivotSetpoint;

	public Arm() {
		extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
		pivotMotor = new CANSparkMax(ArmConstants.PIVOT_MOTOR, MotorType.kBrushless);

		extenderController = ArmConstants.EXTENDER_CONTROLLER;
		pivotController = ArmConstants.PIVOT_CONTROLLER;

		extenderController.setTolerance(.1);

		extendingMotor.setIdleMode(IdleMode.kBrake);

		pivotMotor.getEncoder().setPosition(0);
		pivotMotor.setIdleMode(IdleMode.kBrake);

		potReading = new AnalogInput(Constants.ArmConstants.STRING_POT_CHANNEL);

		PIVOT = ArmConstants.PIVOT;
		EXTENDER = ArmConstants.EXTENDER;

		this.isManual = false;

		restPivot();
		restExtender();


		shuffleboardInit();
	}

	public PIDController getExtenderController() {
		return extenderController;
	}

	public double getPivotSetpoint() {
		return pivotSetpoint;
	}

	public void setPivotSetpoint(double change) {
		pivotSetpoint = change;
	}

	public void setExtenderSetpoint(double change) {
		extenderSetpoint = change;
	}

	public void restPivot() {
		pivotSetpoint = 0;
	}

	public void restExtender() {
		extenderSetpoint = -1.6;
	}

	public void runExtender(double speed) {
		this.isManual = true;
		extendingMotor.set(speed);
	}

	public void runPivot(double speed) {
		this.isManual = true;
		pivotMotor.set(speed);
	}

	private double getExtenderLength() {
		// 1543 is the length pulled out by default, 78 ohms per inch
		return (((double) potReading.getAverageValue() - 1543) / 78);
//		return potReading.getAverageValue(); // DO NOT USE THIS
	}

	public void shuffleboardInit() {
		EXTENDER.addDouble("String Pot Reading UNmodified", () -> potReading.getAverageValue());
		EXTENDER.addDouble("String Pot Reading modified", () -> getExtenderLength());

		EXTENDER.addDouble("Setpoint", () -> extenderSetpoint);

		EXTENDER.addBoolean("At setPoint", () -> extenderController.atSetpoint());
		EXTENDER.addDouble("Setpoint Acording to the PID controller", () -> extenderController.getSetpoint());
		EXTENDER.addDouble("Temp", () -> extendingMotor.getMotorTemperature());

		PIVOT.addDouble("Position", () -> pivotMotor.getEncoder().getPosition());
		PIVOT.addDouble("Setpoint", () -> pivotSetpoint);
		EXTENDER.addDouble("Applied Output", () -> extendingMotor.getAppliedOutput());
		EXTENDER.addDouble("Setpoint bc i do not care", () -> extenderController.getSetpoint());
	}

	@Override
	public void periodic() {
		// limitSetpoint();

		// if (!isManual) {
		// extendingMotor.set(
		// Math.min(Math.max(extenderController.calculate(getExtenderLength(),
		// this.extenderSetpoint), -.5),
		// .5));

		// pivotMotor.set(
		// Math.min(Math.max(
		// pivotController.calculate(this.pivotMotor.getEncoder().getPosition(),
		// this.pivotSetpoint),
		// -.45), .45));
		// }

		extendingMotor.set(
				Math.min(Math.max(extenderController.calculate(getExtenderLength(), this.extenderSetpoint), -.5), .5));

		pivotMotor.set(
				Math.min(Math.max(
						pivotController.calculate(this.pivotMotor.getEncoder().getPosition(), this.pivotSetpoint),
						-.45), .45));
	}

	public void usePID() {
		this.isManual = false;
	}
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
	public final CANSparkMax extendingMotor, pivotMotor;
	private final PIDController extenderController, pivotController;
	private final AnalogInput potReading;
	private final ShuffleboardLayout PIVOT, EXTENDER;
	private boolean isManual;
	private double extenderSetpoint;
	private double pivotSetpoint;
	public Arm() {

		extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
		pivotMotor = new CANSparkMax(ArmConstants.PIVOT_MOTOR, MotorType.kBrushless);
		extenderController = ArmConstants.EXTENDER_CONTROLLER;
		pivotController = ArmConstants.PIVOT_CONTROLLER;

		extenderController.setTolerance(.15);

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

	public double getExtenderSetpoint() {
		return extenderSetpoint;
	}

	public void setExtenderSetpoint(double change) {
		extenderSetpoint = change;
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
		EXTENDER.addDouble("String Pot Reading Unmodified", () -> potReading.getAverageValue());
		EXTENDER.addDouble("String Pot Reading modified", () -> getExtenderLength());

		EXTENDER.addDouble("Setpoint", () -> extenderSetpoint);
		EXTENDER.addBoolean("At setpoint", () -> extenderController.atSetpoint());
		EXTENDER.addDouble("Temperature", () -> extendingMotor.getMotorTemperature());
		EXTENDER.addDouble("Applied Output", () -> extendingMotor.getAppliedOutput());

		PIVOT.addDouble("Position", () -> pivotMotor.getEncoder().getPosition());
		PIVOT.addDouble("Setpoint", () -> pivotSetpoint);
		PIVOT.addDouble("Applied Output", () -> pivotMotor.getAppliedOutput());
		PIVOT.addDouble("Temperature", () -> pivotMotor.getMotorTemperature());
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

		if (RobotState.isEnabled()) {
			extenderSetpoint = Math.max(extenderSetpoint, -3.5);
			extenderSetpoint = Math.min(extenderSetpoint, 21.5);

			pivotSetpoint = Math.min(pivotSetpoint, 0);
			pivotSetpoint = Math.max(pivotSetpoint, 40);

			extendingMotor.set(
					Math.min(Math.max(extenderController.calculate(getExtenderLength(), this.extenderSetpoint), -.55), .55));

			pivotMotor.set(
					Math.min(Math.max(
							pivotController.calculate(this.pivotMotor.getEncoder().getPosition(), this.pivotSetpoint),
							-.45), .45));
		}

	}

	public void usePID() {
		this.isManual = false;
	}
}

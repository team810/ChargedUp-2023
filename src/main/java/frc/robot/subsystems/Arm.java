package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
	public final CANSparkMax extendingMotor, pivotMotor;
	private final PIDController extenderController, pivotController;
	private final AnalogInput potReading;
	private double extenderSetpoint;
	private double pivotSetpoint;

	private boolean isManual;

	public Arm() {

		extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
		pivotMotor = new CANSparkMax(ArmConstants.PIVOT_MOTOR, MotorType.kBrushless);
		extenderController = new PIDController(.25, 0, 0);
		pivotController = new PIDController(0.1, 0, 0);

		extenderController.setTolerance(.15);

		extendingMotor.setIdleMode(IdleMode.kBrake);

		pivotMotor.getEncoder().setPosition(0);
		pivotMotor.setIdleMode(IdleMode.kBrake);

		potReading = new AnalogInput(Constants.ArmConstants.STRING_POT_CHANNEL);

		this.isManual = false;

		restPivot();
		restExtender();
	}

	public double getExtenderSetpoint() {
		return extenderSetpoint;
	}

	public void setExtenderSetpoint(double change) {
		extenderController.reset();
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
		extenderController.reset();
		extenderSetpoint = -2.2;
	}

	private double getExtenderLength() {
		// 1543 is the length pulled out by default, 78 ohms per inch
		return (((double) potReading.getAverageValue() - 1543) / 78);
	}

	@Override
	public void periodic() {
		if (RobotState.isEnabled()) {
			extendingMotor.set(
					Math.min(Math.max(extenderController.calculate(getExtenderLength(), this.extenderSetpoint), -.75), .75));

			pivotMotor.set(
					Math.min(Math.max(
							pivotController.calculate(this.pivotMotor.getEncoder().getPosition(), this.pivotSetpoint),
							-.55), .55));
		}

	}

	public void usePID() {
		this.isManual = false;
	}

	public void runExtender(double speed) {
		this.isManual = true;
		extendingMotor.set(speed);
	}

	public void runPivot(double speed) {
		this.isManual = true;
		pivotMotor.set(speed);
	}

	public boolean extenderAtSetpoint()
	{
		return extenderController.atSetpoint();
	}

	public boolean pivotAtSetpoint()
	{
		return pivotController.atSetpoint();
	}
}

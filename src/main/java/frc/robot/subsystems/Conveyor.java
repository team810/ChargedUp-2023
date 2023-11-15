package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
	public final CANSparkMax conveyorMotor;

	private double speed;
	private int gamePiece = 0; // if set to zero there is no gamePiece 1 is cone and 2 is cube

	private boolean enabled;
	private boolean reversed;
	private boolean scoring;
	private boolean override;

	private boolean manual;

	public Conveyor() {

		conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);

		enabled = false;
		reversed = false;
		scoring = false;
	}

	public void setOvorride(boolean override) {
		this.override = override;
	}

	public boolean getOverride() {
		return override;
	}

	public void runConveyor(double speed) {
		conveyorMotor.set(speed);
	}

	private void updateGamePiece() {
		setGamePiece(0);
		updateMotor();
	}


	void updateMotor() {

		if (enabled) {
			if (!reversed) {
				if (gamePiece == 0 || override) {

					conveyorMotor.set(ConveyorConstants.MOTOR_SPEED);
				} else {
					conveyorMotor.set(0);
				}
			} else {
				conveyorMotor.set(-ConveyorConstants.MOTOR_SPEED);
			}

		} else if (manual) {
			conveyorMotor.set(speed * .25);
		} else {
			if (!scoring) {
				conveyorMotor.set(0);
			}
		}
		// manual = speed < .09 && speed > -.09;
		manual = !(speed == 0);
	}


	private void setGamePiece(int gamePiece) {
		this.gamePiece = gamePiece;
	}

	public boolean isEnabled() {
		return enabled;
	}

	public void setEnabled(boolean enabled) {
		updateMotor();
		this.enabled = enabled;
	}

	public boolean isReversed() {
		return reversed;
	}

	public void setReversed(boolean reversed) {
		updateMotor();
		this.reversed = reversed;
	}

	public boolean isScoring() {
		return scoring;
	}

	public void setScoring(boolean scoring) {
		this.scoring = scoring;
	}

	public double getSpeed() {
		return speed;
	}

	public void setSpeed(double mSpeed) {
		speed = mSpeed;
	}

	@Override
	public void periodic() {
		if(RobotState.isTeleop())
		{
			updateMotor();
		}
		updateGamePiece();
		
	}

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
	public final CANSparkMax conveyorMotor;
	// private final ColorSensor colorSensor;
	// private final Ultrasonic mUltrasonic;


	private final ShuffleboardLayout CONVEYOR_TAB;

	private double speed;
	private int gamePiece = 0; // if set to zero there is no gamePiece 1 is cone and 2 is cube

	private boolean enabled;
	private boolean reversed;
	private boolean scoring;
	private boolean override;

	private boolean manual;

	public Conveyor() {

		conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
		// colorSensor = new ColorSensor();

		CONVEYOR_TAB = ConveyorConstants.CONVEYOR_LAYOUT;

		// mUltrasonic = new Ultrasonic(0, 1);
		// mUltrasonic.setEnabled(true);


		enabled = false;
		reversed = false;
		scoring = false;

		shuffleboardInit();

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

	public void shuffleboardInit() {
		// CONVEYOR_TAB.addDouble("Ultrasonic distance", mUltrasonic::getRangeInches);
		// CONVEYOR_TAB.addDouble("Velocity",
		// () -> conveyorMotor.getEncoder().getVelocity());
		CONVEYOR_TAB.addDouble("Temp", () -> conveyorMotor.getMotorTemperature());

	}

	public int getGamePiece() {
		updateGamePiece();
		return gamePiece;
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
		updateGamePiece();
		updateMotor();
	}

}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
	private final CANSparkMax gripperMotor;
	private final ShuffleboardLayout GRIPPER_MOTOR = GripperConstants.GRIPPER_M_VALUES;

	public Gripper() {
		gripperMotor = new CANSparkMax(GripperConstants.GRIPPER_MOTOR, MotorType.kBrushless);

		gripperMotor.restoreFactoryDefaults();

		gripperMotor.setSmartCurrentLimit(20);

		gripperMotor.setIdleMode(IdleMode.kBrake);
		shuffleboardInit();
	}

	public void setMotor(double speed) {
		gripperMotor.set(speed);
	}

	public void shuffleboardInit() {

		GRIPPER_MOTOR.addDouble("Temp", gripperMotor::getMotorTemperature);

	}
}

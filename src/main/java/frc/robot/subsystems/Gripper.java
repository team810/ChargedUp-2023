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
	//limit switch returns true when not pressed
//	private boolean openGripper, scoring;
//	private final Conveyor m_conveyor;

	//	public Gripper(Conveyor conveyor){
	public Gripper() {
		gripperMotor = new CANSparkMax(GripperConstants.GRIPPER_MOTOR, MotorType.kBrushless);

		gripperMotor.setSmartCurrentLimit(30);

		gripperMotor.setIdleMode(IdleMode.kBrake);


//		this.m_conveyor = conveyor;
//
//		this.openGripper = false;
//		this.scoring = false;

		shuffleboardInit();
	}

	public void setMotor(double speed) {
		gripperMotor.set(speed);
	}

	public void shuffleboardInit() {
		GRIPPER_MOTOR.addDouble("Velocity", () -> gripperMotor.getEncoder().getVelocity());
		GRIPPER_MOTOR.addDouble("Position", () -> gripperMotor.getEncoder().getPosition());
		GRIPPER_MOTOR.addDouble("Temp", gripperMotor::getMotorTemperature);
	}

//	public void gripPiece(Boolean state)
//	{
//		this.openGripper = false;
//		this.scoring = state;
//	}
//	public void openGripper(Boolean state)
//	{
//		this.openGripper = state;
//	}
//	private boolean getGamePiece()
//	{
//		//Same as in conveyor class, 1 for cone, 2 for cube, 0 when there's nothing
//		if(m_conveyor.getGamePiece() == 1)
//			return true;
//		else
//			return false;
//	}
//
//	public void updateGripper()
//	{
//		if(openGripper)
//		{
//
//		}
//		else if(scoring && !openGripper)
//		{
//			if(getGamePiece())
//				//cone
//				gripperMotor.set(.6);
//			else
//				//Cube
//				gripperMotor.set(.3);
//		}
//		else
//			gripperMotor.set(0);
//	}
}

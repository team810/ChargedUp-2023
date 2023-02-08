package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private CANSparkMax gripperMotor;
  private RelativeEncoder encoder;
  private double targetEncoderValue;
  private double rawEncoderValue;
  private double previouseEncoderValue;
  private PIDController controller;

  public Gripper() {
    gripperMotor = new CANSparkMax(GripperConstants.GRIPPER_MOTOR, MotorType.kBrushless);
    encoder = gripperMotor.getEncoder();
    controller = new PIDController(0,0,0); // FIXME tune pid controller
    gripperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    rawEncoderValue = 0;
    previouseEncoderValue = encoder.getPosition();

    targetEncoderValue = GripperConstants.ENCODER_VALUE_OPEN;

    controller.setSetpoint(rawEncoderValue);
  }

  public void runGripper(double speed) {

    this.gripperMotor.set(speed);

  }
  private void updateEncoder()
  {
    rawEncoderValue = rawEncoderValue + (encoder.getPosition() - previouseEncoderValue);
    previouseEncoderValue = encoder.getPosition();
  }

  public void setMode(int mode)
  {
    // mode 1 means cone
    // mode 2 means cube
    // mode 3 is open
    switch (mode)
    {
      case 1:
        targetEncoderValue = GripperConstants.ENCODER_VALUE_CONE;
        break;
      case 2:
        targetEncoderValue = GripperConstants.ENCODER_VALUE_CUBE;
        break;
      case 3:
        targetEncoderValue = GripperConstants.ENCODER_VALUE_OPEN;
        break;
    }
  }

  public void updateMotor()
  {
    controller.setSetpoint(rawEncoderValue);
    gripperMotor.set(controller.calculate(targetEncoderValue));
  }

  public void shuffleboardInit() {
    ShuffleboardTab gripperTab = Shuffleboard.getTab("Gripper");

    gripperTab.addDouble("Gripper Motor Velocity", () -> gripperMotor.getEncoder().getVelocity());
    gripperTab.addDouble("Gripper Position", () -> gripperMotor.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    updateEncoder();
    updateMotor();
  }
}

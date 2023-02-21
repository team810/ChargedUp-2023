// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private final CANSparkMax gripperMotor;
  private final PIDController gripperPIDController;
  private double setPoint;
  private final ShuffleboardLayout GRIPPER_MOTOR = GripperConstants.GRIPPER_M_VALUES;
  private final ShuffleboardLayout GRIPPER_PID = GripperConstants.GRIPPER_M_VALUES;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperMotor = new CANSparkMax(GripperConstants.GRIPPER_MOTOR, MotorType.kBrushless);
    gripperPIDController = GripperConstants.GRIPPER_CONTROLLER;
    gripperPIDController.setTolerance(.2);

    this.setPoint = 0;

    gripperPIDController.reset();

    gripperMotor.clearFaults();
    gripperMotor.restoreFactoryDefaults();
    gripperMotor.setIdleMode(IdleMode.kBrake);

    gripperMotor.setSmartCurrentLimit(20);

    shuffleboardInit();
  }

  public void runGripper(double speed) {
    this.gripperMotor.set(speed);
  }

  public void rest() {
    this.setPoint = -4.5;
  }

  public void gripCube() {
    this.setPoint = 0;
  }

  public void gripCone() {
    this.setPoint = 10;
  }

  public void shuffleboardInit() {
    GRIPPER_MOTOR.addDouble("Velocity", () -> gripperMotor.getEncoder().getVelocity());
    GRIPPER_MOTOR.addDouble("Position", () -> gripperMotor.getEncoder().getPosition());
    GRIPPER_MOTOR.addDouble("Temp", () -> gripperMotor.getMotorTemperature());

    GRIPPER_PID.addDouble("Setpoint", () -> this.setPoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gripperMotor.set(gripperPIDController.calculate(gripperMotor.getEncoder().getPosition(), this.setPoint) * .4);
  }
}

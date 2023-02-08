// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private CANSparkMax gripperMotor;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperMotor = new CANSparkMax(GripperConstants.GRIPPER_MOTOR, MotorType.kBrushless);
  }

  public void runGripper(double speed) {
    this.gripperMotor.set(speed);
  }

  public void shuffleboardInit() {
    ShuffleboardTab gripperTab = Shuffleboard.getTab("Gripper");

    gripperTab.addDouble("Gripper Motor Velocity", () -> gripperMotor.getEncoder().getVelocity());
    gripperTab.addDouble("Gripper Position", () -> gripperMotor.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

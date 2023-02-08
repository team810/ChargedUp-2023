// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax extendingMotor;
  private final CANSparkMax raisingMotor;

  /** Creates a new Arm. */
  public Arm() {
    extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
    raisingMotor = new CANSparkMax(ArmConstants.RAISING_MOTOR, MotorType.kBrushless);
  }

  public void runExtender(double speed) {
    extendingMotor.set(speed);
  }

  public void runRaisingMotor(double speed) {
    raisingMotor.set(speed);
  }

  public void shuffleboardInit() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    armTab.addNumber("Extending Motor Velocity", () -> extendingMotor.getEncoder().getVelocity());
    armTab.addNumber("Raising Motor Velocity", () -> raisingMotor.getEncoder().getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

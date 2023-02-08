// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  private final CANSparkMax conveyorMotor;

  /** Creates a new Conveyor. */
  public Conveyor() {
    conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
  }

  public void runConveyor(double speed) {
    conveyorMotor.set(speed);
  }

  public void shuffleboardInit() {
    ShuffleboardTab conveyorTab = Shuffleboard.getTab("Conveyor");

    conveyorTab.addNumber("Conveyor Motor Velocity", () -> conveyorMotor.getEncoder().getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

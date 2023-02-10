// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax extendingMotor, pivotMotor;
  private final PIDController extenderController, pivotController;
  private final RelativeEncoder pivotEncoder;
  private AnalogInput potReading;
  private double extenderSetpoint, pivotSetpoint;

  public Arm() {
    extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ArmConstants.RAISING_MOTOR, MotorType.kBrushless);

    extenderController = ArmConstants.EXTENDER_CONTROLLER;
    pivotController = ArmConstants.PIVOT_CONTROLLER;

    pivotEncoder = pivotMotor.getEncoder();

    potReading = new AnalogInput(Constants.ArmConstants.STRING_POT_CHANNEL);
  }

  public void rest() {
    pivotSetpoint = 0;
    extenderSetpoint = 0;
  }

  public void middleGoal() {
    pivotSetpoint = 3;
    extenderSetpoint = 3;
  }

  public void highGoal() {
    pivotSetpoint = 6;
    extenderSetpoint = 6;
  }

  private double getExtenderLength() {
    return (potReading.getAverageValue() - 35) / 78;
  }

  public void shuffleboardInit() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    armTab.getLayout("Arm Values", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);

    armTab.getLayout("Arm Values").addNumber("Extender Distance", () -> getExtenderLength());
    armTab.getLayout("Arm Values").addNumber("Extender Setpoint", () -> extenderSetpoint);

    armTab.getLayout("Arm Values").addNumber("Arm Current Height", () -> pivotEncoder.getPosition());
    armTab.getLayout("Arm Values").addNumber("Pivot Setpoint", () -> pivotSetpoint);
  }

  @Override
  public void periodic() {
    extendingMotor.set(extenderController.calculate(getExtenderLength(), extenderSetpoint));
    pivotMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotSetpoint));
  }
}

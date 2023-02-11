// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax extendingMotor;
  private final PIDController extenderController;
  private final PIDController pivotController;
  private final CANSparkMax pivotMotor;
  private final RelativeEncoder pivotEncoder;
  private AnalogInput potReading;
  private double armTargetLength = 0;

  private double targetPivotValue = 0;



  public Arm() {
    extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ArmConstants.RAISING_MOTOR, MotorType.kBrushless);

    extenderController = ArmConstants.EXTENDER_CONTROLLER;
    pivotController = ArmConstants.PIVOT_CONTROLLER;

    pivotEncoder = pivotMotor.getEncoder();

    potReading = new AnalogInput(Constants.ArmConstants.STRING_PLOT_CHANELLE);
  }


  public void setArmTargetLength(double set)
  {
    armTargetLength = set;
  }
  public double getTargetPivotValue()
  {
    return targetPivotValue;
  }
  public void setArmHight(double set) {targetPivotValue = set;}
  public double getArmTargetLength()
  {
    return armTargetLength;
  }
  private void updateExtender()
  {
    pivotMotor.set(extenderController.calculate(getExtenderLength(),armTargetLength));
  }
  private void updatePivot()
  {
    pivotMotor.set(pivotController.calculate(pivotEncoder.getPosition(), targetPivotValue));
  }

  private double getExtenderLength()
  {
    return (potReading.getAverageValue() - 35) / 78;
  }
  public void shuffleboardInit() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    armTab.addNumber("Arm Length", () -> getExtenderLength());
    armTab.addNumber("Arm Target Length", () -> armTargetLength);

    armTab.addNumber("Arm Target Height", () -> targetPivotValue);
    armTab.addNumber("Arm Current Height", () -> pivotEncoder.getPosition()); // This is going to be encoder value
  }

  @Override
  public void periodic() {
    updateExtender();
    updatePivot();
  }
}

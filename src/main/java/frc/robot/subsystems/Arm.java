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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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
  private final ShuffleboardLayout PIVOT, EXTENDER;

  public Arm() {
    extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ArmConstants.PIVOT_MOTOR, MotorType.kBrushless);

    extenderController = ArmConstants.EXTENDER_CONTROLLER;
    pivotController = ArmConstants.PIVOT_CONTROLLER;

    pivotEncoder = pivotMotor.getEncoder();

    potReading = new AnalogInput(Constants.ArmConstants.STRING_POT_CHANNEL);

    PIVOT = ArmConstants.PIVOT;
    EXTENDER = ArmConstants.EXTENDER;
  }

  public void rest() {
    pivotSetpoint = 0;
    extenderSetpoint = 0;
  }

  public void lowGoal()
  {
    pivotSetpoint = 3;
    extenderSetpoint = 3;
  }

  public void middleGoal() {
    pivotSetpoint = 6;
    extenderSetpoint = 6;
  }

  public void highGoal() {
    pivotSetpoint = 12;
    extenderSetpoint = 12;
  }

  private double getExtenderLength() {
    // 35 is the length pulled out by default cus of spacer, 78 ohms per inch
    return (potReading.getAverageValue() - 35) / 78;
  }

  public void shuffleboardInit() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    ShuffleboardLayout extenderLayout = armTab.getLayout("Extender");
    ShuffleboardLayout pivotLayout = armTab.getLayout("Pivot");

    extenderLayout.addDouble("String Pot Reading", () -> getExtenderLength());
    extenderLayout.addDouble("Target Reading", () -> extenderSetpoint);


    pivotLayout.addDouble("Raw Encoder Reading", () -> pivotMotor.getEncoder().getPosition());
    pivotLayout.addDouble("Target Reading", () -> pivotSetpoint);
  }

  @Override
  public void periodic() {
    extendingMotor.set(extenderController.calculate(getExtenderLength(), extenderSetpoint));
    pivotMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotSetpoint));
  }
}

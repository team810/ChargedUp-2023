// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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

    extendingMotor.setIdleMode(IdleMode.kBrake);

    extendingMotor.getEncoder().setPosition(0);

    pivotEncoder = pivotMotor.getEncoder();

    potReading = new AnalogInput(Constants.ArmConstants.STRING_POT_CHANNEL);

    PIVOT = ArmConstants.PIVOT;
    EXTENDER = ArmConstants.EXTENDER;

    extenderSetpoint = 0;
  }

  public void runPivot(double speed)
  {
    pivotMotor.set(speed);
  }

  public void rest() {
    pivotSetpoint = 0;
  }

  public void lowGoal() {
    pivotSetpoint = -10.429;
  }

  public void middleGoal() {
    pivotSetpoint = -24.428;
  }

  public void highGoal() {
    pivotSetpoint = -37.642;
  }

  private double getExtenderLength() {
    // 35 is the length pulled out by default cus of spacer, 78 ohms per inch
    return (potReading.getAverageValue() - 35) / 78;
  }

  public void shuffleboardInit() {
    EXTENDER.addDouble("String Pot Reading", () -> getExtenderLength());
    EXTENDER.addDouble("Setpoint", () -> extenderSetpoint);

    PIVOT.addDouble("Position", () -> pivotMotor.getEncoder().getPosition());
    PIVOT.addDouble("Setpoint", () -> pivotSetpoint);
  }

  @Override
  public void periodic() {
    extendingMotor.set(extenderController.calculate(this.extendingMotor.getEncoder().getPosition(), 0));
    // pivotMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotSetpoint) * .5);
    pivotMotor.set(Math.min(Math.max(pivotController.calculate(this.pivotMotor.getEncoder().getPosition(), this.pivotSetpoint), -.2), .2));
  }
}

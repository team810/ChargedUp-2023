// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

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
    pivotMotor.getEncoder().setPosition(0);
    pivotMotor.restoreFactoryDefaults();
    extendingMotor.restoreFactoryDefaults();

    pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    extendingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    extendingMotor.clearFaults();
    pivotMotor.clearFaults();

    extenderController = ArmConstants.EXTENDER_CONTROLLER;
    pivotController = ArmConstants.PIVOT_CONTROLLER;

    pivotEncoder = pivotMotor.getEncoder();

    potReading = new AnalogInput(Constants.ArmConstants.STRING_POT_CHANNEL);

    PIVOT = ArmConstants.PIVOT;
    EXTENDER = ArmConstants.EXTENDER;



    shuffleboardInit();
  }

  public void rest() {
    pivotSetpoint = 0;
    extenderSetpoint = 0;
    System.out.println("R");
  }

  public void lowGoal() {
    pivotSetpoint = -10.429;
    extenderSetpoint = 3;
    System.out.println("lowG");
  }

  public void middleGoal() {
    pivotSetpoint = -24.428;
    extenderSetpoint = 6;
    System.out.println("mG");
  }

  public void highGoal() {
    pivotSetpoint = -37.642;
    extenderSetpoint = 0;
    System.out.println("hG");
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
//    extendingMotor.set(Math.max(Math.min(extenderController.calculate(getExtenderLength(), -.1), .1), extenderSetpoint));
//    pivotMotor.set(Math.max(Math.min(pivotController.calculate(pivotEncoder.getPosition(),-.2), .2), pivotSetpoint));
    double tmp = Math.max(Math.min(pivotController.calculate(pivotEncoder.getPosition(), pivotSetpoint), 0.2), -0.2);
    pivotMotor.set(tmp);
    System.out.println(tmp);
  }
}

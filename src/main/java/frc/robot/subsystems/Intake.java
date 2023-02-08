// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor;
  private final CANSparkMax rightIntakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);
  }

  public void runIntake(double speed) {
    // The intake motors will always run at the same speed,
    // one of them has to run "backwards" so they are in the same direction
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(-speed);
  }

  public void reverseIntake(double speed) {
    // in case we get something stuck
    leftIntakeMotor.set(-speed);
    rightIntakeMotor.set(speed);
  }

  public void shuffleboardInit() {
    ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

    intakeTab.addDouble("Intake Motor Velocity", () -> leftIntakeMotor.getEncoder().getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

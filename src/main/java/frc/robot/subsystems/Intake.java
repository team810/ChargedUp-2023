// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor, rightIntakeMotor;
  private final String[] states_names = {"Stopped, Running, Running Reversed"};
  public int state;
  private final DoubleSolenoid leftPiston, rightPiston;


  /** Creates a new Intake. */
  public Intake() {
    leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

    leftPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
    rightPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);

    leftPiston.set(DoubleSolenoid.Value.kOff);
    rightPiston.set(DoubleSolenoid.Value.kOff);

    setState(IntakeConstants.STOP_INTAKE);

    shuffleboardInit();
  }

  public void setState(int state)
  {
    this.state = state;
    switch (state)
    {
      case 0:
        stopIntake();
        break;
      case 1:
        runIntake(IntakeConstants.INTAKE_SPEED);
        break;
      case 2:
        reverseIntake(IntakeConstants.INTAKE_SPEED);
        break;
    }

  }

  public void actuateIntake()
  {
    leftPiston.toggle();
    rightPiston.toggle();
  }

  public void stopIntake()
  {
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
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
    intakeTab.getLayout("Motor Values", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    intakeTab.getLayout("Motor Values").addDouble("Velocity", () -> leftIntakeMotor.getEncoder().getVelocity());
    intakeTab.addString("Current State", () -> states_names[state]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  private final CANSparkMax conveyorMotor;
  private final ColorSensor colorSensor;
  private double speed;
  private final ShuffleboardLayout CONVEYOR_TAB;

  /** Creates a new Conveyor. */
  public Conveyor() {
    conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    colorSensor = new ColorSensor();

    conveyorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    CONVEYOR_TAB = ConveyorConstants.CONVEYOR_LAYOUT;

    shuffleboardInit();
  }

  public void runConveyor(double speed) {
    this.speed = speed;
    conveyorMotor.set(speed);
  }

  // To move conveyor during scoring
  public void runConveyorWithColor() {
    if (colorSensor.getColor().equals("Yellow") || colorSensor.getColor().equals("Purple"))
      runConveyor(0);
    else
      runConveyor(.25);
  }

  public void shuffleboardInit() {
    /*CONVEYOR_TAB.getLayout("Motor Values").addDouble("Velocity",
        () -> this.speed);*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runConveyorWithColor();
  }
}

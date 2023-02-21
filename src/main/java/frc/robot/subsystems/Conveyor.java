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
  private int gamePiece = 0; // if set to zero there is no gamePiece 1 is cone and 2 is cube
  private boolean disabled;
  private boolean reversed;

  /** Creates a new Conveyor. */
  public Conveyor() {
    conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    colorSensor = new ColorSensor();

    CONVEYOR_TAB = ConveyorConstants.CONVEYOR_LAYOUT;

    shuffleboardInit();

    disabled = true; // CONVEYOR IS DISABLED BY DEFAULT pls do not debug this
    setReversed(false);
  }

  public void runConveyor(double speed) {
    this.speed = speed;
    conveyorMotor.set(speed);
  }

  // To move conveyor during scoring

  public void runConveyorWithColor() {
    if (colorSensor.getColor().equals("Yellow") || colorSensor.getColor().equals("Purple")) {
      runConveyor(0);
      if (colorSensor.getColor().equals("Yellow")) {
        gamePiece = 1;
      } else if (colorSensor.getColor().equals("Purple")) {
        gamePiece = 2;
      }
    } else {
      runConveyor(ConveyorConstants.MOTOR_SPEED);
      gamePiece = 0;
    }
  }

  public boolean isReversed() {
    return reversed;
  }

  public void setReversed(boolean reversed) {
    this.reversed = reversed;
  }

  public int getGamePiece() {
    return gamePiece;
  }

  public boolean isDisabled() {
    return disabled;
  }

  public void setDisabled(boolean disabled) {
    this.disabled = disabled;
  }

  public void shuffleboardInit() {
    CONVEYOR_TAB.addDouble("Velocity",
        () -> this.speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

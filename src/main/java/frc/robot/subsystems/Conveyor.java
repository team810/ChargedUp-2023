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
  public final CANSparkMax conveyorMotor;
  private final ColorSensor colorSensor;

  private double speed;
  private final ShuffleboardLayout CONVEYOR_TAB;
  private int gamePiece = 0; // if set to zero there is no gamePiece 1 is cone and 2 is cube

  private boolean enabled;
  private boolean reversed;

  /** Creates a new Conveyor. */
  public Conveyor() {
    conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    colorSensor = new ColorSensor();

    CONVEYOR_TAB = ConveyorConstants.CONVEYOR_LAYOUT;

    enabled = false;
    reversed = false;

    shuffleboardInit();

  }

  private void updateGamePiece()
  {
    if (colorSensor.getColor().equals("Yellow"))
    {
      setGamePiece(1);
    } else if (colorSensor.getColor().equals("Purple")) {
      setGamePiece(2);
    }else{
      setGamePiece(0);
    }
    updateMotor();
  }

  void updateMotor()
  {
    if (enabled && gamePiece == 0)
    {
      if (!reversed)
      {
        conveyorMotor.set(ConveyorConstants.MOTOR_SPEED);
      }else{
        conveyorMotor.set(-ConveyorConstants.MOTOR_SPEED);
      }
    }else{
      conveyorMotor.set(0);
    }
  }
  @Override
  public void periodic() {
    updateGamePiece();

  }

  public void shuffleboardInit() {
    CONVEYOR_TAB.addDouble("Velocity",
            () -> this.speed);
  }

  public int getGamePiece() {
    return gamePiece;
  }

  private void setGamePiece(int gamePiece) {
    this.gamePiece = gamePiece;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void setEnabled(boolean enabled) {
    updateMotor();

    this.enabled = enabled;
  }

  public boolean isReversed() {
    return reversed;
  }

  public void setReversed(boolean reversed) {
    updateMotor();
    this.reversed = reversed;
  }
}

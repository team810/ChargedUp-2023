package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor;
  private final CANSparkMax rightIntakeMotor;
  private final String[] states_names = {"Stopped, Running, Running Reversed"};
  public int state;

  public Intake() {
    leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

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

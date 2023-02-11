package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final PIDController extenderController;
  private final CANSparkMax extendingMotor;
  private AnalogInput potReading;
  private double extenderTargetValue;


  private final PIDController pivotController;
  private final CANSparkMax pivotMotor;
  private double pivotContinuesValue;
  private double previousCountDif;
  private double pivotTargetValue; // This is the target encoder value

  public Arm() {
    extendingMotor = new CANSparkMax(ArmConstants.EXTENDING_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ArmConstants.RAISING_MOTOR, MotorType.kBrushless);

    extenderController = ArmConstants.EXTENDER_CONTROLLER;
    pivotController = ArmConstants.PIVOT_CONTROLLER;

    potReading = new AnalogInput(Constants.ArmConstants.STRING_PLOT_CHANELLE);

    pivotMotor.getEncoder().setPosition(0);

    pivotContinuesValue = 0;
    previousCountDif = pivotMotor.getEncoder().getPosition() - pivotContinuesValue;

    pivotTargetValue = ArmConstants.PIVOT_HEIGHT[0];
    extenderTargetValue = ArmConstants.EXTENDER_LENGTHS[0];

    shuffleboardInit();
  }
  public void shuffleboardInit() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    ShuffleboardLayout extenderLayout = armTab.getLayout("Extender");
    ShuffleboardLayout pivotLayout = armTab.getLayout("Pivot");

    extenderLayout.addDouble("String Pot Reading", () -> getPivotTargetValue());
    extenderLayout.addDouble("Target Reading", () -> extenderTargetValue);


    pivotLayout.addDouble("Pivot Continues Reading", () ->pivotContinuesValue);
    pivotLayout.addDouble("Raw Encoder Reading", () -> pivotMotor.getEncoder().getPosition());
    pivotLayout.addDouble("Target Reading", () -> pivotTargetValue);
  }

  public void setTarget(int target[])
  {
    setPivotTargetValue(ArmConstants.PIVOT_HEIGHT[target[0]]);
    setExtenderTargetValue(ArmConstants.EXTENDER_LENGTHS[target[1]]);
  }

  private void updateExtender()
  {
    double speed = extenderController.calculate(getExtenderLength(), getExtenderTargetValue());
    extendingMotor.set(speed);
  }
  public void updatePivot()
  {
    double speed = pivotController.calculate(pivotContinuesValue,getPivotTargetValue());
    pivotMotor.set(speed);
  }

  @Override
  public void periodic() {
    pivotContinuesValue = pivotContinuesValue + previousCountDif;

    updatePivot();
    updateExtender();
  }

  private double getExtenderLength()
  {
    return (potReading.getAverageValue() - 35) / 78;
  }

  public double getPivotTargetValue() {
    return pivotTargetValue;
  }

  public void setPivotTargetValue(double pivotTargetValue) {
    this.pivotTargetValue = pivotTargetValue;
  }

  public double getExtenderTargetValue() {
    return extenderTargetValue;
  }

  public void setExtenderTargetValue(double extenderTargetValue) {
    this.extenderTargetValue = extenderTargetValue;
  }
}

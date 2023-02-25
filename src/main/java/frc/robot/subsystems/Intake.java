
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor, rightIntakeMotor;
  private final DoubleSolenoid piston;
  private final ShuffleboardLayout INTAKE_VALUES = IntakeConstants.INTAKE_VALUES;
  private final PneumaticHub pneumaticHub;


  /** Creates a new Intake. */
  public Intake() {
    leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

    pneumaticHub = new PneumaticHub(18);

    pneumaticHub.clearStickyFaults();


    pneumaticHub.enableCompressorDigital();

    piston = pneumaticHub.makeDoubleSolenoid(0, 7);

    shuffleboardInit();
  }

  public void out()
  {
    piston.set(Value.kForward);
  }
  public void in()
  {
    piston.set(Value.kReverse);
  }

  public void runIntake(double speed) {
    leftIntakeMotor.set(-speed);
    rightIntakeMotor.set(speed);
  }

  public void toggleIntake()
  {
    this.piston.toggle();
  }

  public void shuffleboardInit() {
    INTAKE_VALUES.addBoolean("Compressor On?", ()-> this.pneumaticHub.getCompressor());
    INTAKE_VALUES.addDouble("Velocity", () -> leftIntakeMotor.getEncoder().getVelocity());
//    INTAKE_VALUES.add("sol", pHub);
  }

  public Value getSolenoidValue() {
    return piston.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
  }
}

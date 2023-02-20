
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor, rightIntakeMotor;
  private final DoubleSolenoid piston;
  private final ShuffleboardLayout INTAKE_VALUES = IntakeConstants.INTAKE_VALUES;
  private final PneumaticHub pHub;


  /** Creates a new Intake. */
  public Intake() {
    leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    pHub = new PneumaticHub(18);
    pHub.clearStickyFaults();

    
    pHub.enableCompressorDigital();

    piston = pHub.makeDoubleSolenoid(0, 7);
    // piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 7);
    // piston.set(DoubleSolenoid.Value.kOff);

    
    

  }

  public void runIntake(double speed) {
//     The intake motors will always run at the same speed,
//     one of them has to run "backwards" so they are in the same direction
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(-speed);
  }

  public void setForward() {
    piston.set(Value.kForward);
    System.out.println("PISTONS SET FORWARD");
  }

  public void setBackward() {
    piston.set(Value.kReverse);
    System.out.println("PISTONS SET REVERSED");
  }

  public void shuffleboardInit() {
    INTAKE_VALUES.addBoolean("Compressor On?", ()-> this.pHub.getCompressor());
    INTAKE_VALUES.addDouble("Velocity", () -> leftIntakeMotor.getEncoder().getVelocity());
  }

  public Value getSolenoidValue() {
    return piston.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    // System.out.println(pHub.getCompressor());
    
  }
}

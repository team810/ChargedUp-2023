
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

    private Boolean scoring;


    /**
     * Creates a new Intake.
     */
    public Intake() {
        leftIntakeMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        rightIntakeMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

        pneumaticHub = new PneumaticHub(18);

        pneumaticHub.clearStickyFaults();


        pneumaticHub.enableCompressorDigital();

        piston = pneumaticHub.makeDoubleSolenoid(0, 7);
        scoring = false;
        shuffleboardInit();
    }

    public void out() {
        if (!scoring)
        {
            piston.set(Value.kForward);
        }else{
            piston.set(Value.kReverse);
        }
    }

    public void in() {
        piston.set(Value.kReverse);
    }

    public void runIntake() {
        leftIntakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
        rightIntakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void stopIntake() {
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }

    public void runIntakeReversed() {
        leftIntakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
        rightIntakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
    }


    public void toggleIntake() {
        this.piston.toggle();
    }

    public void shuffleboardInit() {
        INTAKE_VALUES.addDouble("Velocity", () -> leftIntakeMotor.getEncoder().getVelocity());
    }

    public Value getSolenoidValue() {
        return piston.get();
    }

    public Boolean getScoring() {
        return scoring;
    }

    public void setScoring(Boolean mScoring) {
        scoring = mScoring;
    }
}

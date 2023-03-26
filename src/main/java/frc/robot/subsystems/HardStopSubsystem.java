package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class HardStopSubsystem implements Subsystem {
	private final DoubleSolenoid solenoid;

	public HardStopSubsystem() {
		solenoid = Constants.PNEUMATIC_HUB.makeDoubleSolenoid(0, 7);

		Shuffleboard.getTab("Hard Stop").addBoolean("Hard Stop State", () -> solenoid.get() == Value.kForward);
		Shuffleboard.getTab("Hard Stop").addBoolean("Compressor On", () -> Constants.PNEUMATIC_HUB.getCompressor());
	}

	public void out() {
		solenoid.set(Value.kForward);
	}

	public void in() {
		solenoid.set(Value.kReverse);
	}

    public void toggle()
    {
        if (solenoid.get() == Value.kForward)
        {
			solenoid.set(Value.kReverse);
        }else{
			solenoid.set(Value.kForward);
        }
    }
}


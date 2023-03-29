package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class HardStopSubsystem implements Subsystem {
	private final DoubleSolenoid solenoid;

	public HardStopSubsystem() {
		solenoid = Constants.PNEUMATIC_HUB.makeDoubleSolenoid(0, 7);

		Shuffleboard.getTab("Hard Stop").addBoolean("Hard Stop State", () -> solenoid.get() == DoubleSolenoid.Value.kForward);
		Shuffleboard.getTab("Hard Stop").addBoolean("Compressor On", () -> Constants.PNEUMATIC_HUB.getCompressor());
	}

	public void out() {
		solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void in() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void toggle() {
		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			solenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			solenoid.set(DoubleSolenoid.Value.kForward);
		}
	}
}

//

/*
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class HardStopSubsystem implements Subsystem {
	Compressor m_compressor;
	DoubleSolenoid sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);
	public HardStopSubsystem()
	{
		m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
		m_compressor.enableDigital();

		sol.set(DoubleSolenoid.Value.kForward);
	}

	public void out() {
		sol.set(DoubleSolenoid.Value.kReverse);
	}

	public void in() {
		sol.set(DoubleSolenoid.Value.kForward);
	}

	public void toggle() {
		if (sol.get() == DoubleSolenoid.Value.kForward)
		{
			sol.set(DoubleSolenoid.Value.kReverse);
		}else{
			sol.set(DoubleSolenoid.Value.kForward);
		}
	}
}



*/
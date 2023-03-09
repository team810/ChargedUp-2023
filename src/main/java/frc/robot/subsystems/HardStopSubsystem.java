package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class HardStopSubsystem implements Subsystem {
    private final DoubleSolenoid solenoid;

    public HardStopSubsystem()
    {
        solenoid = Constants.PNEUMATIC_HUB.makeDoubleSolenoid(Constants.HardStop.LEFT_SOL, Constants.HardStop.RIGHT_SOL);
    }

    public void out()
    {
        solenoid.set(Value.kForward);
    }

    public void in()
    {
        solenoid.set(Value.kReverse);
    }
}


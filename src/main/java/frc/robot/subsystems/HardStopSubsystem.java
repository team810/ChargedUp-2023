package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class HardStopSubsystem implements Subsystem {
    private final Solenoid leftSol;
    private final Solenoid rightSol;

    public HardStopSubsystem()
    {
        leftSol = Constants.PNEUMATIC_HUB.makeSolenoid(Constants.HardStop.LEFT_SOL);
        rightSol = Constants.PNEUMATIC_HUB.makeSolenoid(Constants.HardStop.RIGHT_SOL);
    }

    public void out()
    {
        leftSol.set(true);
        rightSol.set(true);
    }

    public void in()
    {
        leftSol.set(false);
        rightSol.set(false);
    }
    public void toggle()
    {
        leftSol.toggle();
        rightSol.toggle();
    }
}


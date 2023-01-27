package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    private boolean State;
    Solenoid sol = new Solenoid(PneumaticsModuleType.REVPH,1);
    MotorController left = new CANSparkMax(Constants.Intake.INTEAKE_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    MotorController right = new CANSparkMax(Constants.Intake.INTEAKE_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private MotorControllerGroup motorGroup = new MotorControllerGroup(left, right);

    private IntakeSubsystem(Supplier<Boolean> Extened) {
        sol.set(false);
    }
    public boolean GetState()
    {
        return State;
    }
    public void ToggleState()
    {
        State = !State;
        Update();
    }
    public void SetState(boolean NewState)
    {
        State = NewState;
        Update();
    }
    private void Update()
    {
        if (State == true)
        {
            motorGroup.set(Constants.Intake.Speed);
        }else{
            motorGroup.set(0);
        }
    }
}


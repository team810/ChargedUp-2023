package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extender extends SubsystemBase {

    private final CANSparkMax extenderMotor;
    private PIDController pidController;
    private double targetLength = 0;
    private AnalogInput potReading;

    public Extender()
    {
        extenderMotor = new CANSparkMax(Constants.ArmConstants.EXTENDER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        pidController = new PIDController(0,0,0); // FIXME needs to be tuned
        potReading = new AnalogInput(Constants.ArmConstants.STRING_PLOT_CHANELLE);
        targetLength = 0;

        pidController.setSetpoint(targetLength);
    }


    private void update()
    {
        pidController.setSetpoint(targetLength);
        extenderMotor.set(pidController.calculate(getCurrentLength()));
    }

    public double getCurrentLength()
    {
        return (potReading.getAverageValue() - 35) / 78;
    }
    public void setTargetLength(double targetLength)
    {
        this.targetLength = targetLength;
    }
    public double getTargetLength()
    {
        return targetLength;
    }

    @Override
    public void periodic() {
        update();
    }
}


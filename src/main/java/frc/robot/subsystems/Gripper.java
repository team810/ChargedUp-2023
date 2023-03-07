package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
    private final CANSparkMax gripperMotor;
    private final PIDController gripperPIDController;
    private double setPoint;

    private final Solenoid solenoid;

    private final ShuffleboardLayout GRIPPER_MOTOR = GripperConstants.GRIPPER_M_VALUES;
    private final ShuffleboardLayout GRIPPER_PID = GripperConstants.GRIPPER_M_VALUES;

    private final DigitalOutput limit_switch;

    public Gripper() {
        gripperMotor = new CANSparkMax(GripperConstants.GRIPPER_MOTOR, MotorType.kBrushless);
        gripperPIDController = GripperConstants.GRIPPER_CONTROLLER;
        gripperPIDController.setTolerance(.1);
        this.setPoint = 0;

        gripperPIDController.reset();

        gripperMotor.clearFaults();
        gripperMotor.restoreFactoryDefaults();
        gripperMotor.setSmartCurrentLimit(20);
        
        gripperMotor.setIdleMode(IdleMode.kBrake);

        gripperMotor.getEncoder().setPosition(0);


        solenoid = Constants.PNEUMATIC_HUB.makeSolenoid(GripperConstants.GRIPPER_SOL);

        limit_switch = new DigitalOutput(GripperConstants.LIMIT_SWITCH);


        shuffleboardInit();
    }


    public void openGripper() {
        setPoint(-1.5);
    }

    public void closeGripper() {
        setPoint(0);
    }

    public void gripCube() {
        setPoint(3);
    }

    public void gripCone() {
        setPoint(10);
    }

    public void setPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getSetpoint()
    {
        return this.setPoint;
    }

    public void shuffleboardInit() {
        GRIPPER_MOTOR.addDouble("Velocity", () -> gripperMotor.getEncoder().getVelocity());
        GRIPPER_MOTOR.addDouble("Position", () -> gripperMotor.getEncoder().getPosition());
        GRIPPER_MOTOR.addDouble("Temp", gripperMotor::getMotorTemperature);
        GRIPPER_PID.addDouble("Setpoint", () -> this.setPoint);
    }

    @Override
    public void periodic() {
        if (limit_switch.get())
        {
            gripperMotor.getEncoder().setPosition(0);
        }
        gripperMotor.set(gripperPIDController.calculate(gripperMotor.getEncoder().getPosition(), this.setPoint));
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Conveyor;

import java.util.function.DoubleSupplier;

public class ConveyorCommand extends CommandBase {
	private final Conveyor conveyor;
	private final DoubleSupplier m_speed;

	public ConveyorCommand(Conveyor conveyor, DoubleSupplier speed) {
		this.conveyor = conveyor;
		m_speed = speed;

		addRequirements(this.conveyor);
	}

	@Override
	public void initialize() {
		super.initialize();
	}

	@Override
	public void execute() {
		conveyor.setSpeed(m_speed.getAsDouble() * .25);
	}

	@Override
	public boolean isFinished() {
		return super.isFinished();
	}
}

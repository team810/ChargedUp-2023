package lib;


import edu.wpi.first.math.MathUtil;

public class Deadband {

	private final double deadBand; // The dead band range

	public Deadband(double deadBand) {
		this.deadBand = deadBand;
	}

	public double apply(double input) {
		return MathUtil.applyDeadband(input, deadBand);
	}
}

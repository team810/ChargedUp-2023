package lib;


import edu.wpi.first.math.MathUtil;

public class Deadband {

	private final double deadBand; // The dead band range
	private final double defaultValue; // The default value when input is within the dead band

	public Deadband(double deadBand, double defaultValue) {
		this.deadBand = deadBand;
		this.defaultValue = defaultValue;
	}

	public double apply(double input) {
		return MathUtil.applyDeadband(input, deadBand);
	}
}

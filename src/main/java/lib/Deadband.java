package lib;


public class Deadband {

	private final double deadBand; // The dead band range
	private final double defaultValue; // The default value when input is within the dead band

	public Deadband(double deadBand, double defaultValue) {
		this.deadBand = deadBand;
		this.defaultValue = defaultValue;
	}

	public double apply(double input) {
		if (Math.abs(input) <= deadBand) {
			return defaultValue; // Input within the dead band, return the default value
		} else {
			// Add your own logic here to calculate the output outside the dead band
			// For example, you can return the input value itself or any other desired output.
			// Replace the line below with the desired logic:
			return input;
		}
	}
}

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorSensorConstants;

public class ColorSensor extends SubsystemBase {
	private final ColorSensorV3 m_colorSensorV3;

	private final ColorMatch m_colorMatcher;
	private final Color kYellowTarget = new Color(0.355, 0.555, 0.090);

	private final Color kPurpleTarget = new Color(0.203, 0.317, 0.479);
	private final ShuffleboardLayout COLOR_TAB = ColorSensorConstants.COLOR_SENSOR;
	private Color detectedColor;
	private double IR;
	private String colorString;
	private ColorMatchResult match;

	public ColorSensor() {
		m_colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);
		m_colorMatcher = new ColorMatch();

		m_colorMatcher.setConfidenceThreshold(.90);

		m_colorMatcher.addColorMatch(kYellowTarget);
		m_colorMatcher.addColorMatch(kPurpleTarget);

		this.detectedColor = m_colorSensorV3.getColor();
		this.IR = m_colorSensorV3.getIR();
		this.match = m_colorMatcher.matchClosestColor(detectedColor);
		colorString = "Unknown";

		shuffleInit();
	}

	public Color getYellow() {
		return this.kYellowTarget;
	}

	public Color getPurple() {
		return this.kPurpleTarget;
	}

	public Color getDetectedColor() {
		return this.detectedColor;
	}

	public String getColor() {
		return this.colorString;
	}

	private void shuffleInit() {

		COLOR_TAB.addDouble("Green", () -> detectedColor.green);
		COLOR_TAB.addDouble("Blue", () -> detectedColor.blue);
		COLOR_TAB.addDouble("Confidence", () -> match.confidence);
		COLOR_TAB.addDouble("IR", () -> IR);
		COLOR_TAB.addString("Detected Color", () -> colorString);
		COLOR_TAB.addDouble("Red", () -> detectedColor.red);
	}

	@Override
	public void periodic() {
		this.detectedColor = m_colorSensorV3.getColor();
		this.IR = m_colorSensorV3.getIR();
		ColorMatchResult localMatch = m_colorMatcher.matchColor(detectedColor);
		if (localMatch != null) {
			this.match = localMatch;

			if (match.color == kYellowTarget) {
				colorString = "Yellow";
			} else if (match.color == kPurpleTarget) {
				colorString = "Purple";
			} else {
				colorString = "Unknown";
			}
		} else {
			colorString = "Unknown";
		}
	}

}

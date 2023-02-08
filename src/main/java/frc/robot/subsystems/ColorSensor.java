package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private final ColorSensorV3 m_colorSensorV3;
    private final ColorMatch m_colorMatcher;

    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
    private final Color kPurpleTarget = new Color(0.2, 0.312, 0.488);

    private Color detectedColor;
    private double IR;
    private String colorString;
    private ColorMatchResult match;

    public ColorSensor() {
        m_colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);
        m_colorMatcher = new ColorMatch();

        m_colorMatcher.addColorMatch(kYellowTarget);
        m_colorMatcher.addColorMatch(kPurpleTarget);

        shuffleInit();
    }

    private void shuffleInit() {
        ShuffleboardTab colorsensorTab = Shuffleboard.getTab("Color Sensor");

        colorsensorTab.getLayout("Color Sensor", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0);

        colorsensorTab.add("Red", detectedColor.red);
        colorsensorTab.add("Green", detectedColor.green);
        colorsensorTab.add("Blue", detectedColor.blue);

        colorsensorTab.add("Confidence", match.confidence);

        colorsensorTab.add("IR", IR);

        colorsensorTab.add("Detected Color", colorString);
    }

    @Override
    public void periodic() {
        this.detectedColor = m_colorSensorV3.getColor();
        this.IR = m_colorSensorV3.getIR();
        this.match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else if (match.color == kPurpleTarget) {
            colorString = "Purple";
        } else {
            colorString = "Unknown";
        }
    }
}

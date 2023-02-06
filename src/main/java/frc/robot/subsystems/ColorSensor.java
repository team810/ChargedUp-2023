package frc.robot.subsystems;


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private final ColorSensorV3 m_colorSensorV3;
    private final ColorMatch m_colorMatcher;

    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
    private final Color kPurpleTarget = new Color(0.2, 0.312, 0.488);

    public ColorSensor() {
        m_colorSensorV3  = new ColorSensorV3(I2C.Port.kOnboard);
        m_colorMatcher = new ColorMatch();

        m_colorMatcher.addColorMatch(kYellowTarget);
        m_colorMatcher.addColorMatch(kPurpleTarget);
    }

    @Override
    public void periodic() {
        Color detectedColor = m_colorSensorV3.getColor();

        double IR = m_colorSensorV3.getIR();
        /**
         * Run the color match algorithm on our detected color
         */
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else if(match.color == kPurpleTarget){
            colorString = "Purple";
        } else {
            colorString = "Unknown";
        }

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);

        SmartDashboard.putNumber("IR", IR);

        SmartDashboard.putString("Detected Color", colorString);
    }
}


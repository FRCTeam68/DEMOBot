/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.revrobotics.ColorMatch;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;

public class DJSpinner extends SubsystemBase {
  /**
   * Creates a new DJSpinner.
   */
 // private final WPI_TalonSRX spinner;

 // private final ColorSensorV3 sensor;

  private static DJSpinner djSpinner;
  
/*
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private final ColorMatch m_colorMatcher = new ColorMatch();
  private String colorString;
*/

  public static DJSpinner getDJSpinner() {
    if (djSpinner == null) {
      djSpinner = new DJSpinner();
    }
    return djSpinner;
  }
/*
  public DJSpinner() {
    spinner = new WPI_TalonSRX(Constants.SPINNER_MOTOR);
    spinner.setNeutralMode(NeutralMode.Brake);

    spinner.configPeakOutputForward(1);
    spinner.configPeakOutputReverse(-1);

    sensor = new ColorSensorV3(Constants.i2cPort);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);  
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  //  Color detectedColor = sensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    /*
     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
  public String SensorColor(){
    return colorString;
  }
  public void spinMotor(double speed){
    spinner.set(speed);
  }
  */
}
}

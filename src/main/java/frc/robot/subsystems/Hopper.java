/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  WPI_VictorSPX agitator1;
  WPI_VictorSPX agitator2;

  public Hopper() {
  agitator1 = new WPI_VictorSPX(Constants.HOPPER_AGITATOR1);
  agitator2 = new WPI_VictorSPX(Constants.HOPPER_AGITATOR2);
  agitator1.setSensorPhase(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAgitatorSpeed(double speed, double speed2){
    agitator1.set(speed);
    agitator2.set(speed2);
  }
}

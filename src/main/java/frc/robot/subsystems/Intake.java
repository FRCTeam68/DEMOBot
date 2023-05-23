/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Robot;
import frc.robot.commands.*;

public class Intake extends SubsystemBase {

  private WPI_VictorSPX intakeMotor;
  
  public static Intake intake;
  /**
   * Creates a new Intake.
   */
  public static Intake getIntake(){
    if(intake == null){
      intake = new Intake();
    }
    return intake;
  } 
  public Intake() {
    intakeMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR);
    intakeMotor.configPeakOutputForward(1);
    intakeMotor.configPeakOutputReverse(-1);

  }



  @Override
  public void periodic() {
    CommandScheduler.getInstance().setDefaultCommand(Robot.intake, new IntakeCommand());
    // This method will be called once per scheduler run
  }
public void setIntakeSpeed(double speed) {
  intakeMotor.set(speed);

}

}

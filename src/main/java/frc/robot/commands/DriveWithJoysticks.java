/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;

public class DriveWithJoysticks extends CommandBase {
  /**
   * Creates a new DriveWithJoysticks.
   */
  double leftStick;
  double rightStick;
  double rightSpeed;
  double leftSpeed;
  public DriveWithJoysticks() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.pnuematics.gearMode()){
    leftStick = -Robot.m_robotContainer.getLeftXboxJoystickValue()*Math.abs(Robot.m_robotContainer.getLeftXboxJoystickValue());
    rightStick = Robot.m_robotContainer.getRightXboxJoystickValueX()*Math.abs(Robot.m_robotContainer.getRightXboxJoystickValueX());
    } else {
      leftStick = -Robot.m_robotContainer.getLeftXboxJoystickValue()*Math.abs(Robot.m_robotContainer.getLeftXboxJoystickValue());
      rightStick = Robot.m_robotContainer.getRightXboxJoystickValueX()*Math.abs(Robot.m_robotContainer.getRightXboxJoystickValueX())*.5;
    }
    if(rightStick > 0 || leftStick == leftStick){
      rightSpeed = leftStick - rightStick;
      leftSpeed = leftStick + rightStick;
    } else if(rightStick < 0 || leftStick == leftStick){
      leftSpeed = leftStick + rightStick;
      rightSpeed = leftStick - rightStick;
    } else {
      leftSpeed = 0;
      rightSpeed = 0;
    }
    if(Robot.m_robotContainer.getXboxDriveRB() == true){ 
      DriveTrain.getDriveTrain().setSpeedFalcon(Robot.driveTrain.leftVisionAdjusted(),Robot.driveTrain.rightVisionAdjusted());
    }
    else{

      DriveTrain.getDriveTrain().setSpeedFalcon(0.75*leftSpeed, 0.75*rightSpeed);  
      }
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

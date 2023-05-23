/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SpinToColor extends CommandBase {
  /**
   * Creates a new SpinToColor.
   */
  public SpinToColor() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   /*
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
      if(Robot.djspinner.SensorColor() != "Blue"){
        Robot.djspinner.spinMotor(1);
      } else if(Robot.djspinner.SensorColor() == "Blue"){
        Robot.djspinner.spinMotor(0);
      }
        break;
      case 'G':
      if(Robot.djspinner.SensorColor() != "Green"){
        Robot.djspinner.spinMotor(1);
      } else if(Robot.djspinner.SensorColor() == "Green"){
        Robot.djspinner.spinMotor(0);
      }
        // Green case code
        break;
      case 'R':
      if(Robot.djspinner.SensorColor() != "Red"){
        Robot.djspinner.spinMotor(1);
      } else if(Robot.djspinner.SensorColor() == "Red"){
        Robot.djspinner.spinMotor(0);
      }
        // Red case code
        break;
      case 'Y':
      if(Robot.djspinner.SensorColor() != "Yellow"){
        Robot.djspinner.spinMotor(1);
      } else if(Robot.djspinner.SensorColor() == "Yellow"){
        Robot.djspinner.spinMotor(0);
      }
        // Yellow case code
        break;
      default:
        // This is corrupt data
        break;
      }
    } else {
      Robot.djspinner.spinMotor(0);
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

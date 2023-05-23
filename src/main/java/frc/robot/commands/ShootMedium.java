/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShootMedium extends CommandBase {
  /**
   * Creates a new ShootMedium.
   */
  private Boolean finished = false;
  public ShootMedium() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.shooter.setShooterAngle(Constants.SHOOTER_MEDIUM_TICKS);
    Robot.shooter.setShooterVelocity(Constants.SHOOTER_MEDIUM_SPEED_LEFT,Constants.SHOOTER_MEDIUM_SPEED_RIGHT, Constants.SHOOTER_FEEDER_SPEED);
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

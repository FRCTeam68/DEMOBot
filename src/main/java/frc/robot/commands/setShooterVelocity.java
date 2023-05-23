/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class setShooterVelocity extends CommandBase  {
  /**
   * Creates a new setShooterVelocity.
   */

  public setShooterVelocity() {
    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(Robot.smartPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // SmartDashboard.putNumber("rotations", SmartDashboard.getNumber("kRotations", 0));
    //Robot.smartPID.setShooterVelocity(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    SmartDashboard.putNumber("rotations", Robot.smartPID.getEntrySetPoint());
    System.out.print("BUTTON PRESSED()");
    
    Robot.shooter.setShooterPID(Robot.smartPID.getEntryP(),
    Robot.smartPID.getEntryI(),
    Robot.smartPID.getEntryD(),
    Robot.smartPID.getEntryF(),
    Robot.smartPID.getEntryP_2(),
    Robot.smartPID.getEntryI_2(),
    Robot.smartPID.getEntryD_2(),
    Robot.smartPID.getEntryF_2());
  
    Robot.shooter.setShooterVelocity(Robot.smartPID.getEntrySetPoint(),Robot.smartPID.getEntrySetPoint_2());

    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.commands.AutonLock;
import frc.robot.commands.AutonLockZero;
import frc.robot.commands.ChangeIntakePos;
import frc.robot.commands.LiftAdd1Deg;
import frc.robot.commands.LiftMinus1Deg;
import frc.robot.commands.SetAgitator;
import frc.robot.commands.ShiftGears;
import frc.robot.commands.ShootLow;
import frc.robot.commands.ShootMedium;
import frc.robot.commands.SpinFeeder;
import frc.robot.commands.Zero;
import frc.robot.loops.SubsystemManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.2
 */
// import frc.robot.commands.DriveWithJoysticks;
// import frc.robot.subsystems.DriveTrain;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SubsystemManager manager;
public enum EnableState {
    DISABLED,
    AUTON,
    TELEOP
}
  public EnableState enableState = EnableState.DISABLED;

  XboxController xboxDrive;
  XboxController xboxManipulator;
  JoystickButton xboxDriveRT;
  JoystickButton xboxDriveRB;
  JoystickButton xboxManipX;
  JoystickButton xboxManipCircle;
  JoystickButton xboxManipRS;
  JoystickButton xboxManipSquare;
  JoystickButton xboxManipTriangle;
  JoystickButton xboxManipLT;
  JoystickButton xboxManipRT;


  //Command shiftGear = new ShiftGears();

  private static RobotContainer robotContainer;

  public static RobotContainer getRobotContainer() {
    if (robotContainer == null) {
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }
  public Command getAutonomousCommand() {
    return null;
    //return new RunAutonStraightMeter(Robot.robotOdemetry, Robot.driveTrain);
  }

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    manager = new SubsystemManager(0.0);
    //xboxDriveRT.whenPressed(new ShiftGears());
    xboxManipX.whileTrue(new ShootLow());
    xboxManipCircle.whileTrue(new ShootMedium());
    xboxManipTriangle.whileTrue(new ShootLow());
    xboxManipRS.onFalse(new ChangeIntakePos());
    xboxManipSquare.whileTrue(new Zero());
    //xboxManipX.whenReleased(new Zero());
    xboxManipCircle.onFalse(new Zero());
    xboxManipTriangle.onFalse(new Zero());
    //xboxManipLT.whenPressed(new LiftMinus1Deg());
    xboxManipRT.whileTrue(new SpinFeeder(.75));
    xboxManipRT.onFalse(new SpinFeeder(0));

  }

  /** 
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxDrive = new XboxController(Constants.XBOX_DRIVE);
    xboxManipulator = new XboxController(Constants.XBOX_MANIPULATE);
    xboxDriveRB = new JoystickButton(xboxDrive, Constants.XBOX_DRIVE_RB);
    xboxDriveRT = new JoystickButton(xboxDrive, Constants.XBOX_DRIVE_RT_BUTTON);
    xboxManipX = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_X);
    xboxManipCircle = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_CIRCLE);
    xboxManipRS = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_SR);
    xboxManipSquare = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_SQUARE);
    xboxManipTriangle = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_TRIANGLE);
    xboxManipLT = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_LT);
    xboxManipRT = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_RT);
    

  }

  public double getLeftXboxJoystickValue() {
    double leftAxis;
    leftAxis = xboxDrive.getLeftY();

    // Allow for up to 10% of joystick noises\
    leftAxis = (Math.abs(leftAxis) < 0.1) ? 0 : leftAxis;
    return leftAxis;
  }

  // Drivetrain Tank Drive Right
  public double getRightXboxJoystickValue() {
    double rightAxis;
    rightAxis = xboxDrive.getRightY();
    // Allow for up to 10% of joystick noise
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
    return rightAxis;
  }
  public double getRightXboxJoystickValueX() {
    double rightAxis;
    
    rightAxis = xboxDrive.getRawAxis(2);
    
    // Allow for up to 10% of joystick noise
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
    return rightAxis;
  }

  public double getLeftXboxManipulatorJoystickValue() {
    double leftAxis;
    leftAxis = xboxManipulator.getLeftY();
    // Allow for up to 10% of joystick noises\
    leftAxis = (Math.abs(leftAxis) < 0.1) ? 0 : leftAxis;
    return leftAxis;
  }

  // Drivetrain Tank Drive Right
  public double getRightXboxManipulatorJoystickValue() {
    double rightAxis;
    rightAxis = xboxManipulator.getRightY();
    // Allow for up to 10% of joystick noise
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
    return rightAxis;

  }


  public boolean getXboxDriveRB() {
		boolean buttonPressed = (xboxDrive.getRightBumper()) ? true : false;
		
		return buttonPressed;
	}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @throws IOException
   */

}

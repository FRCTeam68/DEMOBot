/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DJSpinner;
//import frc.robot.commands.auton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.EndGame;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pnuematics;
//import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

//import frc.robot.subsystems.DriveTrainAuton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static RobotContainer m_robotContainer;

  public static DriveTrain driveTrain;

  public static Pnuematics pnuematics;

  public static Vision vision;

  public static EndGame endGame;

  public static Hopper hopper;

  public static Intake intake;

  public static Shooter shooter;

  public static SmartPID smartPID;

  public static DJSpinner djspinner;
  
  //public static RobotOdometry robotOdemetry;

  Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override

  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    driveTrain = new DriveTrain();
    m_robotContainer = new RobotContainer();
    vision = new Vision();
    pnuematics = new Pnuematics();
    shooter = new Shooter();
    smartPID = new SmartPID();
    intake = new Intake();
    hopper = new Hopper();
    endGame = new EndGame();
    djspinner = new DJSpinner();
    //robotOdemetry = new RobotOdometry();
    /*
     * endGame = new EndGame(); hopper = new Hopper(); intake = new Intake();
     * shooter = new Shooter();
     */

    driveTrain.resetYaw();
    shooter.zeroEncoders();
    driveTrain.breakMode();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    driveTrain.coastMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    driveTrain.breakMode();

    driveTrain.ResetEncoders();
    driveTrain.resetYaw();
    /*
    if(!Constants.isHighGear){
    pnuematics.setShifterHigh();
    } else{
      pnuematics.setShiftLow();
    }
    */
    autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("HEADING IN RADS", (Robot.driveTrain.getYAW() - 180) * (3.141592654 / 180));
    SmartDashboard.putNumber("HEADING IN DEGS", Robot.driveTrain.getYAW());
    SmartDashboard.putBoolean("Is high gear", Constants.isHighGear);
  }

  @Override
  public void teleopInit() {
    driveTrain.breakMode();

    driveTrain.ResetEncoders();
    shooter.zeroEncoders();
    driveTrain.resetYaw();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putNumber("HEADING IN RADS", (Robot.driveTrain.getYAW() - 180) * (3.141592654 / 180));
    //SmartDashboard.putNumber("HEADING IN DEGS", Robot.driveTrain.getYAW());
    SmartDashboard.putBoolean("Target", Robot.vision.getTarget());
    //SmartDashboard.putNumber("Right Enc", Robot.driveTrain.getRightEnc());
    //SmartDashboard.putNumber("Left Enc", Robot.driveTrain.getLeftEnc());
    SmartDashboard.putBoolean("Is high gear", Constants.isHighGear);
    /*
    if (Robot.m_robotContainer.getXboxDriveRB() == true) {
      SmartDashboard.putNumber("Distance to Target", Robot.vision.calcDistance());
    } else {
      SmartDashboard.putNumber("Distance to Target", 0);
    }
    */
    // SmartDashboard.putNumber("LimelightX", Robot.vision.getXValue());
    // SmartDashboard.putNumber("LimelightY", Robot.vision.getYValue());
    // SmartDashboard.putNumber("LimelightArea", Robot.vision.getArea());

    // (SmartDashboard.getKeys());
  }

  // Robbie was here

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

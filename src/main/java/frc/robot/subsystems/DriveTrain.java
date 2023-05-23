/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;

import java.util.function.BooleanSupplier;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoysticks;
//import jdk.vm.ci.meta.Constant;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */ 
  private WPI_TalonFX fr; //front right
  private WPI_TalonFX br; //back right
  private WPI_TalonFX bl; //back left
  private WPI_TalonFX fl; //front left\
  
  
  /*
  private CANCoder leftDriveEnc;
  private CANCoder RightDriveEnc;
  */

  private final DifferentialDriveOdometry m_odometry;

  
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  
  private DriveControlMode currentControlMode = DriveControlMode.STANDARD_DRIVE;
  private Boolean driveDisableSwitchAccess;

  private double wheelDiameter;
  private double encLeftTicks;
  private double encRightTicks;

  protected double maxVelocityLow;
  protected double maxVelocityHigh;
  protected double minVelocityLow;
  protected double minVelocityHigh;
// The robot's drive

// The left-side drive encoder


// The right-side drive encoder

// The gyro sensor
private AHRS m_gyro = new AHRS();

  public static DriveTrain driveTrain;


	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}
  public DriveTrain() {
    // NAVX CONFIG
    m_gyro = new AHRS(SPI.Port.kMXP);
    //m_gyro.reset();
    driveDisableSwitchAccess = false; //false || it attempted to run im fairly certain this has to be set to false so it falls into close loop
    
    if(Constants.isHighGear){
      wheelDiameter = Constants.WHEEL_DIAMETER_HIGH;
      encLeftTicks = Constants.ENCODER_TICK_LEFT_REVOLUTION_HIGH;
      encRightTicks = Constants.ENCODER_TICK_RIGHT_REVOLUTION_HIGH;
    } else {
      wheelDiameter = Constants.WHEEL_DIAMETER_LOW;
      //wheelDiameter = Constants.WHEEL_DIAMETER_LOW_HYPERDRIVE; // ONLY USE THIS WHEN IN HYPERDRive
      encLeftTicks = Constants.ENCODER_TICK_LEFT_REVOLUTION;
      encRightTicks = Constants.ENCODER_TICK_RIGHT_REVOLUTION;
    }

    // DriveTrain Motors Config
    fr = new WPI_TalonFX(Constants.TALONFX_FR);
    br = new WPI_TalonFX(Constants.TALONFX_BR);
    bl = new WPI_TalonFX(Constants.TALONFX_BL);
    fl = new WPI_TalonFX(Constants.TALONFX_FL);
    /*
    CAN ENCODER GETS DEGREES NOT ENCODER TICKS
    leftDriveEnc = new CANCoder(Constants.CANENCODER_LEFT_DRIVE);
    RightDriveEnc = new CANCoder(Constants.CANENCODER_RIGHT_DRIVE);
    */
    fr.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
    fr.selectProfileSlot(Constants.DRIVETRAIN_RIGHT_PID_SLOT, 0);
    fr.setSensorPhase(true);
    
    fr.setInverted(true);
    br.setInverted(true);

    fl.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
    fl.selectProfileSlot(Constants.DRIVETRAIN_RIGHT_PID_SLOT, 0);
    fl.setSensorPhase(false);
    //fl.setInverted(true);
    

    bl.set(ControlMode.Follower, fl.getDeviceID());
    br.set(ControlMode.Follower, fr.getDeviceID());

    fr.config_kP(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_P);
    fr.config_kI(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_I);
    fr.config_kD(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_D);
    fr.config_kF(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_F);

    fl.config_kP(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_P);
    fl.config_kI(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_I);
    fl.config_kD(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_D);
    fl.config_kF(Constants.DRIVETRAIN_RIGHT_PID_SLOT, Constants.DRIVETRAIN_RIGHT_PID_F);

    fr.setNeutralMode(NeutralMode.Brake);
    br.setNeutralMode(NeutralMode.Brake);
    fl.setNeutralMode(NeutralMode.Brake);
    bl.setNeutralMode(NeutralMode.Brake);


    fr.configPeakOutputForward(1);
    br.configPeakOutputForward(1);
    fl.configPeakOutputForward(1);
    bl.configPeakOutputForward(1);
    fr.configPeakOutputReverse(-1);
    br.configPeakOutputReverse(-1);
    bl.configPeakOutputReverse(-1);
    fl.configPeakOutputReverse(-1);

    //m_odometry.resetPosition(, m_gyro.getYaw());
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), encLeftTicks, encLeftTicks);
    

  }



  @Override
  public void periodic() {
    CommandScheduler.getInstance().setDefaultCommand(Robot.driveTrain, new DriveWithJoysticks());
    m_odometry.update(Rotation2d.fromDegrees(getYAW()), encLeftTicks/(3.14159*wheelDiameter/12), Robot.driveTrain.getRightEnc()/encRightTicks/(3.14159*wheelDiameter/12));

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    SmartDashboard.putNumber("Left Encoder Drive", getLeftEnc());
    SmartDashboard.putNumber("Right Encoder Drive", getRightEnc());
    
    //SmartDashboard.putNumber("m_xEntry", m_xEntry);
    SmartDashboard.putNumber("kS", fl.getMotorOutputVoltage());
    //SmartDashboard.putNumber("kV", fl.getMotor)
    SmartDashboard.putNumber("DISTANCE TRAVELED",Robot.driveTrain.getLeftEnc()/(encLeftTicks/(3.14159*wheelDiameter/12)) + Robot.driveTrain.getRightEnc()/encRightTicks/(3.14159*wheelDiameter/12) / 2.0);
    SmartDashboard.putNumber("Drive R veloc", rightVelocity());
    SmartDashboard.putNumber("Drive L veloc", leftVelocity());

  }

  public void setSpeedFalcon(double left, double right){
    
    fl.set(ControlMode.PercentOutput,left);
    fr.set(ControlMode.PercentOutput,right);
    
  }
  public void setSpeedAuto(double left, double right){
   /*
    fl.set(ControlMode.Velocity,left*5000);
    br.set(ControlMode.Velocity,right*5000);
    
    /*
    fl.set(ControlMode.Velocity,left*200);
    br.set(ControlMode.Velocity,right*200);
    */
  }


  /**
   * Zeroes the heading of the robot.
   */
  public void ResetEncoders(){
    fl.setSelectedSensorPosition(0,0,0);
    fr.setSelectedSensorPosition(0,0,0);
    
        /*m_gyro.reset(); for auton experimental we may have to reset thee navx to re run a new path */
  }

  public void resetYaw(){
    //m_gyro.reset();
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from and degree
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  public double getYAW() {
    return m_gyro.getYaw();
  }
  public boolean isNavXReady(){
    return m_gyro.isCalibrating();
  }

  
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getLeftEnc(){
    return fl.getSelectedSensorPosition(0);
  }
  public double getRightEnc(){
    return fr.getSelectedSensorPosition(0);

  }
  public double leftVelocity(){
    return fl.getSelectedSensorVelocity();
  }
  public double rightVelocity(){
    return fr.getSelectedSensorVelocity();
  }
  
  public double getRotationsLeft() {
    return (double) fl.getSelectedSensorPosition() / encLeftTicks;
  }

    
  public double getRotationsRight() {
    return (double) fr.getSelectedSensorPosition() / encRightTicks;
  }
  public double leftVisionAdjusted(){
    double leftSpeed;
    double joystickSpeed;
    double visionCorrect;
    joystickSpeed = Robot.m_robotContainer.getLeftXboxJoystickValue();
    visionCorrect = Robot.vision.steeringAdjust();
    leftSpeed = joystickSpeed += visionCorrect;
    return leftSpeed;
  }
  public double rightVisionAdjusted(){
    double rightSpeed;
    double joystickSpeed;
    double visionCorrect;
    joystickSpeed = Robot.m_robotContainer.getRightXboxJoystickValue();
    visionCorrect = Robot.vision.steeringAdjust();
    rightSpeed =  joystickSpeed -= visionCorrect;
    return rightSpeed;
  }
    /**
   * Gets the distance travelled of the right side of the drive since the last
   * call to resetPosition.
   * 
   * @return Distance in inches
   */
  public double getDistanceRight() {
    return wheelDiameter * Math.PI * getRotationsRight();
  }

  /**
   * Gets the distance travelled of the left side of the drive since the last call
   * to resetPosition.
   * 
   * @return Distance in inches
   */
  public double getDistanceLeft() {
    return wheelDiameter * Math.PI * getRotationsLeft();
  }
  
  public void driveOpenLoopLowLevel(double left, double right) {
    fl.set(ControlMode.PercentOutput, left);
    fr.set(ControlMode.PercentOutput, right);
  }

  
  public void driveClosedLoopLowLevel(double left, double right) {
    fl.set(ControlMode.Velocity, left * encLeftTicks / 10);
    fr.set(ControlMode.Velocity, right * encRightTicks / 10);
  }

  public void stop(){
    fl.set(0);
    fr.set(0);
  }

  private double calcActualVelocity(double input, boolean isPercentage) {
    double minVelocity;
    if (Robot.pnuematics.gearMode()) {
      minVelocity = minVelocityLow;
    } else {
      minVelocity = minVelocityHigh;
    }
    double minNonZero = 0.1;
    if (isPercentage) {
      minVelocity /= Constants.MAX_SPEED;
      minNonZero = 0.001;
    }
    if (input > minNonZero * -1 && input < minNonZero) {
      return 0;
    } else if (input >= minNonZero && input < minVelocity) {
      return minVelocity;
    } else if (input <= minNonZero * -1 && input > minVelocity * -1) {
      return minVelocity * -1;
    } else {
      return input;
    }
  }
  public void driveInchesPerSec(int left, int right) {
    driveInchesPerSec((double) right, (double) left);
  }
  public void driveInchesPerSec(double left, double right) {
    if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
      if (driveDisableSwitchAccess) {
        left = 0;
        right = 0;
      }

      if (Constants.openLoop) {
        driveOpenLoopLowLevel(calcActualVelocity(left, false) / Constants.MAX_SPEED,
            calcActualVelocity(right, false) / Constants.MAX_SPEED);
      } else {
        driveClosedLoopLowLevel((calcActualVelocity(left, false) / (wheelDiameter * Math.PI)),
            (calcActualVelocity(right, false) / (wheelDiameter * Math.PI)));
      }
    }
  }
  
  public void coastMode(){
    fl.setNeutralMode(NeutralMode.Coast);
    fr.setNeutralMode(NeutralMode.Coast);
    br.setNeutralMode(NeutralMode.Coast);
    bl.setNeutralMode(NeutralMode.Coast);
  }
  public void breakMode(){
    fl.setNeutralMode(NeutralMode.Brake);
    fr.setNeutralMode(NeutralMode.Brake);
    br.setNeutralMode(NeutralMode.Brake);
    bl.setNeutralMode(NeutralMode.Brake);
  }
  
  public double getRPSRight() {
    return (double) fr.getSelectedSensorVelocity() / encRightTicks * 10;
  }

  
  public double getRPSLeft() {
    return (double) fl.getSelectedSensorVelocity() / encLeftTicks * 10;
  }
  public double getVelocityLeft() {
    return wheelDiameter * Math.PI * getRPSLeft();
  }
  public double getVelocityRight() {
    return wheelDiameter * Math.PI * getRPSRight();
  }

  private enum DriveControlMode {
    STANDARD_DRIVE, PTO
  }
}
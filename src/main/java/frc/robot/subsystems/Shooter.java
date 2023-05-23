/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private CANSparkMax shooterWheel1;
  private CANSparkMax shooterWheel2;
  private WPI_TalonSRX shooterAngle;
  private WPI_VictorSPX feeder;
  private RelativeEncoder shooterWheel1Enc;
  private RelativeEncoder shooterWheel2Enc;
  private SparkMaxPIDController pidController1;
  private SparkMaxPIDController pidController2;
  /*
  private SimpleMotorFeedforward smFF_pid1;
  private SimpleMotorFeedforward smFF_pid2;
  */
  private DigitalInput limitSwitch;

  public Shooter() {
    limitSwitch = new DigitalInput(Constants.SHOOTER_LIMIT_SWITCH);

    shooterWheel1 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_1, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(Constants.SHOOTER_WHEELSPINNER_2, MotorType.kBrushless);
    shooterWheel1.restoreFactoryDefaults();
    shooterWheel2.restoreFactoryDefaults();
    
    feeder = new WPI_VictorSPX(Constants.SHOOTER_FEEDER);
    shooterAngle = new WPI_TalonSRX(Constants.SHOOTER_ANGLE);
    shooterAngle.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    shooterAngle.selectProfileSlot(Constants.SHOOTER_PID_SLOT, 0);

    shooterAngle.setSensorPhase(true);
    feeder.setSensorPhase(false);

    shooterAngle.config_kP(Constants.SHOOTER_PID_SLOT, Constants.SHOTOER_ANGLE_KP);
    shooterAngle.config_kF(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_ANGLE_KF);

    shooterWheel1Enc = shooterWheel1.getEncoder();
    shooterWheel2Enc = shooterWheel2.getEncoder();
    pidController1 = shooterWheel1.getPIDController();
    pidController2 = shooterWheel2.getPIDController();

    pidController1.setP(Constants.SHOOTER_PID_P);
    pidController1.setI(Constants.SHOOTER_PID_I);
    pidController1.setD(Constants.SHOOTER_PID_D);
    pidController1.setFF(Constants.SHOOTER_PID_F);
    pidController2.setP(Constants.SHOOTER_PID_P);
    pidController2.setI(Constants.SHOOTER_PID_I);
    pidController2.setD(Constants.SHOOTER_PID_D);
    pidController2.setFF(Constants.SHOOTER_PID_F);

    pidController2.setOutputRange(-1, 1);
    pidController1.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //CommandScheduler.getInstance().setDefaultCommand(Robot.shooter, new SetShooterCustom());
    SmartDashboard.putNumber("Velocity", shooterWheel1Enc.getVelocity());
    SmartDashboard.putNumber("Velocity 2", shooterWheel2Enc.getVelocity());
    SmartDashboard.putNumber("Angle Enc", shooterAngle.getSelectedSensorPosition());

  }


  public void setShooterSpeed(double shooterSpeed) {
    shooterWheel1.set(-shooterSpeed);
    shooterWheel2.set(shooterSpeed);
    feeder.set(.5);
  }
  public void setShooterAngle(double angle){
    shooterAngle.set(ControlMode.Position,angle);
  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }
  

  public void setShooterVelocity(double shooterVelocity, double shooterVelocity_2, double feederSpeed) {
    // the value for pidController1 is - because it is backward
    pidController1.setReference(-shooterVelocity, ControlType.kVelocity);
    pidController2.setReference(shooterVelocity_2, ControlType.kVelocity);
   // feeder.set(.5);
  }
  public void setFeederSpeed(double speed){
    feeder.set(speed);
  }
  public void setFeederZero(){
    feeder.set(0);
  }
  public void zeroEncoders(){
    shooterAngle.setSelectedSensorPosition(10);
    feeder.set(0);
  }
  public double getEncoderTicks(){
    return shooterAngle.getSelectedSensorPosition();
  }
}

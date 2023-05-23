/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Controller Inputs
    public static final boolean isHighGear = false;

    public static final int XBOX_DRIVE = 0;

    public static final int XBOX_DRIVE_X = 2;
    public static final int XBOX_DRIVE_CIRCLE = 3;
    public static final int XBOX_DRIVE_SQUARE = 1;
    public static final int XBOX_DRIVE_TRIANGLE = 4;
    public static final int XBOX_DRIVE_LY = 1; // left joystick
    public static final int XBOX_DRIVE_LT = 3;
    public static final int XBOX_DRIVE_LT_BUTTON = 7;
    public static final int XBOX_DRIVE_RT = 4;
    public static final int XBOX_DRIVE_RT_BUTTON = 8;
    public static final int XBOX_DRIVE_RY = 5; // right joystick
    public static final int XBOX_DRIVE_SL = 11;
    public static final int XBOX_DRIVE_SR = 12;
    public static final int XBOX_DRIVE_RB = 6;
    public static final int XBOX_DRIVE_LB = 5;
    public static final int XBOX_DRIVE_SHARE = 9;
    public static final int XBOX_DRIVE_OPTIONS = 10;
    public static final int XBOX_DRIVE_POV_DOWN = 180;
    public static final int XBOX_DRIVE_POV_RIGHT = 90;
    public static final int XBOX_DRIVE_POV_LEFT = 270;
    public static final int XBOX_DRIVE_POV_UP = 0;

    public static final int XBOX_MANIPULATE = 1;

    public static final int XBOX_MANIPULATE_X = 2;
    public static final int XBOX_MANIPULATE_CIRCLE = 3;
    public static final int XBOX_MANIPULATE_SQUARE = 1;
    public static final int XBOX_MANIPULATE_TRIANGLE = 4;
    public static final int XBOX_MANIPULATE_LY = 1; // left joystick
    public static final int XBOX_MANIPULATE_LT = 7;
    public static final int XBOX_MANIPULATE_RT = 8;
    public static final int XBOX_MANIPULATE_RY = 5; // right joystick
    public static final int XBOX_MANIPULATE_SL = 11;
    public static final int XBOX_MANIPULATE_SR = 12;
    public static final int XBOX_MANIPULATE_RB = 6;
    public static final int XBOX_MANIPULATE_LB = 5;
    public static final int XBOX_MANIPULATE_SHARE = 9;
    public static final int XBOX_MANIPULATE_OPTIONS = 10;
    public static final int XBOX_MANIPULATE_POV_DOWN = 180;
    public static final int XBOX_MANIPULATE_POV_RIGHT = 90;
    public static final int XBOX_MANIPULATE_POV_LEFT = 270;
    public static final int XBOX_MANIPULATE_POV_UP = 0;

    // pid values provided by the almight An
    public static final double PID_F = 0.03; // 0.025
    public static final double PID_P = .4;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.0;
    //shooter velocity
    public static final double SHOOTER_PID_F = 0.001; // 0.025
    public static final double SHOOTER_PID_P = 0.0;
    public static final double SHOOTER_PID_I = 0.0;
    public static final double SHOOTER_PID_D = 0.0;


    // drive motor values

    public static final int TALONFX_FR = 11;
    public static final int TALONFX_FL = 13;
    public static final int TALONFX_BR = 12;
    public static final int TALONFX_BL = 14;

    // CAN ENCODER
    public static final int CANENCODER_LEFT_DRIVE = 0;
    public static final int CANENCODER_RIGHT_DRIVE = 0;

    
    // Drivetrain Right PID Config


    public static final int DRIVETRAIN_RIGHT_PID_SLOT = 0;
    public static final double DRIVETRAIN_RIGHT_PID_F = 0.05;//.45
    public static final double DRIVETRAIN_RIGHT_PID_P = 0.00;//.815
    public static final double DRIVETRAIN_RIGHT_PID_I = 0.00;
    public static final double DRIVETRAIN_RIGHT_PID_D = 0.000; //0.1

    public static final double DRIVETRAIN_RIGHT_SLOT = 0;
    public static final double DRIVETRAIN_LEFT_SLOT = 0;

    // auton
    public static final double WHEEL_DIAMETER_LOW = 7; //in inches
    public static final double WHEEL_DIAMETER_LOW_HYPERDRIVE = 7;
    public static final double WHEEL_DIAMETER_HIGH = 6;
    public static final double MAX_SPEED = 17;
    public static final int ENCODER_TICK_LEFT_REVOLUTION = 39834; // -39834      
    public static final int ENCODER_TICK_RIGHT_REVOLUTION = 39834; // 39834
    public static final int ENCODER_TICK_LEFT_REVOLUTION_HIGH = 20036; // -39834      
    public static final int ENCODER_TICK_RIGHT_REVOLUTION_HIGH = 20036; // 39834
    


    public static final boolean openLoop = false; // was true a second ago this inquires wheter or not the robot is in closed loop or not

    // pneumatics constants

    public static final int DRIVE_SHIFTER_PCM_A = 1;
    public static final int DRIVE_SHIFTER_PCM_B = 5;
    public static final int INTAKE_PCM_A = 2;
    public static final int INTAKE_PCM_B = 3;
    public static final int AIR_PUMP_CAN = 0;

    // shooter subsystem

    public static final int SHOOTER_WHEELSPINNER_1 = 6;
    public static final int SHOOTER_WHEELSPINNER_2 = 2;
    public static final int SHOOTER_ANGLE = 8;
    public static final int SHOOTER_FEEDER = 3;
    public static final int SHOOTER_PID_SLOT = 0;
    public static final int SHOOTER_LIMIT_SWITCH = 0;
    public static final double SHOOTER_ANGLE_KF = 0.08;//.02
    public static final double SHOTOER_ANGLE_KP = .4;//.01
    public static final double SHOOTER_MEDIUM_TICKS = -5500;
    public static final double SHOOTER_LOW_TICKS = -10000;
    public static final double SHOOTER_CURVE_SIDE = 1000;
    public static final double SHOOTER_CURVE_OTHER = 200;
    public static final double SHOOTER_LOW_SPEED_LEFT = -457.5;
    public static final double SHOOTER_LOW_SPEED_RIGHT = -357.5;
    public static final double SHOOTER_MEDIUM_SPEED_LEFT = -550;
    public static final double SHOOTER_MEDIUM_SPEED_RIGHT = -450;
    public static final double SHOOTER_FEEDER_SPEED = .5;

    //endgame 20
    public static final int ENDGAME_WINCH = 1;
    public static final int ENDGAME_SERVO = 0;
    
    //Hopper subsystem
    public static final int HOPPER_AGITATOR1 = 4;//current value unknown
    public static final int HOPPER_AGITATOR2 = 10;//current value unknown


    //Intake Subsystem
    public static final int INTAKE_MOTOR = 5;
    //spinner subsystem
    public static final int SPINNER_MOTOR = 0;
    //dj spinner sub
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;

}

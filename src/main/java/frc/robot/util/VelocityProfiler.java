// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Ramps up and down to setpoint for velocity closed loop control */
public class VelocityProfiler {
  private static final double dt = 0.02;
  private final double dv;
  private double currentSetpoint = 0;
  private double goalSetpoint = 0;

  /**
   * Creates a new VelocityProfiler
   * 
   * @param acceleration RPM per second
   */
  public VelocityProfiler(double acceleration) {
    dv = acceleration * dt;
  }

  /**
   * Sets the target setpoint
   * 
   * @param setpoint Target setpoint
   */
  public void setSetpointGoal(double setpoint) {
    goalSetpoint = setpoint;
  }

  /**
   * Sets the target setpoint, starting from the current speed
   * 
   * @param setpoint     Target setpoint
   * @param currentSpeed Current speed, to be used as the starting setpoint
   */
  public void setSetpointGoal(double setpoint, double currentSpeed) {
    goalSetpoint = setpoint;
    currentSetpoint = currentSpeed;
  }

  /**
   * Resets target setpoint and current setpoint
   */
  public void reset() {
    currentSetpoint = 0;
    goalSetpoint = 0;
  }

  /**
   * Returns the current setpoint to send to motors
   * 
   * @return Setpoint to send to motors
   */
  public double getSetpoint() {
    if (goalSetpoint > currentSetpoint) {
      currentSetpoint += dv;
      if (currentSetpoint > goalSetpoint) {
        currentSetpoint = goalSetpoint;
      }
    } else if (goalSetpoint < currentSetpoint) {
      currentSetpoint -= dv;
      if (currentSetpoint < goalSetpoint) {
        currentSetpoint = goalSetpoint;
      }
    }
    return currentSetpoint;
  }

  /**
   * Returns the current setpoint goal
   */
  public double getSetpointGoal() {
    return goalSetpoint;
  }
}

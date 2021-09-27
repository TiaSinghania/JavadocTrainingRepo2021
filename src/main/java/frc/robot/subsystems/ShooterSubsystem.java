// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.HardwareMap.ShooterHardware;

public class ShooterSubsystem extends SubsystemBase {
  private SpeedControllerGroup m_flywheelMotor;
  private SpeedControllerGroup m_feeder;
  private CANEncoder m_flywheelEncoder;
  private double m_targetSpeed;
  private double m_feederSpeed;
  public boolean m_isAtMaxSpeed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ShooterHardware shooterHardware) {
    m_flywheelMotor = shooterHardware.flywheel;
    m_feeder = shooterHardware.feeder;
    m_flywheelEncoder = shooterHardware.rightCanEncoder;
  }

  /**
   * Inherits from SpeedController.set(double) method.
   * 
   * @param power Value from [-1, 1].
   */
  public void setFlywheelPower(double power) {
    this.m_targetSpeed = power;
  }

  /**
   * 
   * @return Speed of flywheel, in RPM
   */
  public double getFlywheelRPM() {
    return m_flywheelEncoder.getVelocity();
  }

  /**
   * Turns on the feeder to feed balls into the shooter. ONLY runs the feeder if
   * the flywheel is up to speed.
   */
  public void turnFeederOn() {
    if (getFlywheelRPM() >= ShooterConstants.FLYWHEEL_READY_RPM) {
      m_feederSpeed = 1;
    } else {
      turnFeederOff();
    }
  }

  /**
   * Turns off the feeder to stop feeding balls into the shooter.
   */
  public void turnFeederOff() {
    m_feederSpeed = 0;
  }

  @Override
  public void periodic() {
    // Spit out the shooter speed
    SmartDashboard.putNumber("Current Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Feeder Speed", m_feederSpeed);
    m_flywheelMotor.set(m_targetSpeed);
    m_feeder.set(m_feederSpeed);
  }
}

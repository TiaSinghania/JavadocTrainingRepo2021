// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareMap.IntakeHardware;

public class IntakeSubsystem extends SubsystemBase {

  private VictorSPX m_intakeController;
  private VictorSPX m_armController;
  private double m_desiredIntakeSpeed;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeHardware intake) {
    // fill this in based on hardwaremap
    m_intakeController = intake.intakeController;
    m_armController = intake.armController;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Speed", m_desiredIntakeSpeed);
    m_intakeController.set(VictorSPXControlMode.PercentOutput, m_desiredIntakeSpeed);

  }

  /**
   * A <b>positive</b> `speed` value makes the arm move <b>up</b>
   * 
   * @param velocity A value from [-1, 1], the speed at which the arm moves
   */
  public void moveArm(double velocity) {
    m_armController.set(VictorSPXControlMode.PercentOutput, velocity);
    SmartDashboard.putNumber("Desired Arm Motor Speed", velocity);
  }

  /**
   * Spins the intake to intake balls.
   */
  public void intake() {
    m_desiredIntakeSpeed = Constants.IntakeConstants.INTAKE_SPEED;
  }

  /**
   * Stops the intake from spinning.
   */
  public void stopIntake() {
    m_desiredIntakeSpeed = 0;
  }

  /**
   * Spins the intake "backwards" to outtake balls.
   */
  public void outtake() {
    m_desiredIntakeSpeed = -Constants.IntakeConstants.INTAKE_SPEED;
  }
}

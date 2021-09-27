// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Utils;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap.ClimberHardware;

public class ClimberSubsystem extends SubsystemBase {
  private Servo m_servoMotor;
  private Servo m_ratchetServo;
  private CANSparkMax m_winchMotor;

  public ClimberSubsystem(ClimberHardware climberHardware) {
    m_servoMotor = climberHardware.servoMotor;
    m_ratchetServo = climberHardware.ratchetServo;
    m_winchMotor = climberHardware.winchMotor;
    
    
    // Set the servo to "return/rest" position at the very beginning.
    this.resetClimber();
  }

  /**
   * @return the angle of the servo that releases the climber.
   */
  public double getPos() {
    return m_servoMotor.get();
  }

  /**
   * Sets the ratchet servo to allow the climb mechanism to extend.
   */
  public void releaseRatchet() {
    m_ratchetServo.set(Constants.ClimberConstants.WINCH_RELEASE_SERVO_POSITION);
    SmartDashboard.putString("Ratchet State", "extend");
    DriverStation.reportError("climb direction reversed", false);
  }

  /**
   * Sets the ratchet servo to only allow the climb mechanism to retract - it is
   * unable to extend.
   */
  public void lockRatchet() {
    m_ratchetServo.set(Constants.ClimberConstants.WINCH_LOCK_SERVO_POSITION);
    SmartDashboard.putString("Ratchet State", "retract");
    DriverStation.reportError("climb direction normal", false);
  }

  /**
   * Moves the Servo to release the Climber mechanism
   */
  public void releaseClimber() {
    m_servoMotor.set(Constants.ClimberConstants.SERVO_RELEASE_POSITION);
    SmartDashboard.putString("ClimberState", "Released");
    DriverStation.reportError("climb released ", false);
  }

  public void resetClimber() {
    m_servoMotor.set(Constants.ClimberConstants.SERVO_RETURN_POSITION);
  }

  /**
   * Moves the servo to the position where the climber would be secured down
   */
  public void lockServo() {
    m_servoMotor.set(Constants.ClimberConstants.SERVO_RETURN_POSITION);
  }

  /**
   * To retract the climber, input a value in [-1, 0]. To extend the climber,
   * input a value in [0, 1]. <b>MAKE SURE THE RACHET IS RELEASED BEFORE INPUTING
   * POSITIVE VALUES</b>, otherwise the motor will stall.
   * 
   * @param speed Speed of the winch motor.
   */
  public void climb(double speed) {
    if (!(Utils.tolerance(m_ratchetServo.get(), Constants.ClimberConstants.WINCH_LOCK_SERVO_POSITION,
        0.1) == Constants.ClimberConstants.WINCH_LOCK_SERVO_POSITION && speed > 0)) {
      m_winchMotor.set(speed);
    } else {
      m_winchMotor.set(0);
    }

  }

  /**
   * @return The speed of the winch motor in RPM
   */
  public double getSpeed() {
    return m_winchMotor.getEncoder().getVelocity();
  }

  public void periodic() {
    SmartDashboard.putNumber("Servo Position", this.getPos());

  }
}

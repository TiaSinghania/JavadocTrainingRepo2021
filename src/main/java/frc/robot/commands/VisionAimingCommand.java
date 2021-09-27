// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class VisionAimingCommand extends SwerveJoystickCommand {
  private final PIDController m_pid = new PIDController(0.03, 0, 0);
  
  /** Creates a new {@link VisionAimingCommand}. */
  public VisionAimingCommand(SwerveDriveSubsystem subsystem, XboxController controller) {
    super(subsystem, controller);
  }

  @Override
  public void initialize() {
    m_pid.reset();
    Limelight.setLed(3);
    m_pid.setSetpoint(0.0); // 0.0 means the limelight is pointed at the right direction

  }
  //@Override
  public double getRotation() { 
    SmartDashboard.putNumber("vision PID", Limelight.getX());

    return m_pid.calculate(Limelight.getX());
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Limelight.setLed(1);
  } 
}

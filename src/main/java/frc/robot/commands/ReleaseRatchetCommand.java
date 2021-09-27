// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorBoard.OperatorBoardButton;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Releases ratchet while command is active. Command does NOT require climber
 * subsystem. Locks ratchet while command is inactive.
 */
public class ReleaseRatchetCommand extends CommandBase {
  private ClimberSubsystem m_subsystem;
  private OperatorBoardButton m_button;

  /** Creates a new ReleaseRatchetCommand. */
  public ReleaseRatchetCommand(ClimberSubsystem subsystem, OperatorBoardButton button) {
    m_subsystem = subsystem;

    m_button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.releaseRatchet();
    m_button.turnLightOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.lockRatchet();
    m_button.turnLightOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

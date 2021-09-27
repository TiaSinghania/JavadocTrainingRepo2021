// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorBoard.OperatorBoardButton;
import frc.robot.subsystems.ClimberSubsystem;


public class ReleaseRatchetCommand extends CommandBase {
  private ClimberSubsystem m_subsystem;
  private OperatorBoardButton m_button;

  public ReleaseRatchetCommand(ClimberSubsystem subsystem, OperatorBoardButton button) {
    m_subsystem = subsystem;

    m_button = button;
  }

  @Override
  public void initialize() {
    m_subsystem.releaseRatchet();
    m_button.turnLightOn();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.lockRatchet();
    m_button.turnLightOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
